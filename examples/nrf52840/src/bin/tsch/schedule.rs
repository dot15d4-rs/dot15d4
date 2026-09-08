use dot15d4::{
    constants::MAC_HOPPING_SEQUENCE_MAX_LENGTH,
    driver::{
        radio::config::Channel,
        socs::nrf::{NrfRadioDriver, NrfRadioSleepTimer},
    },
    mac::{
        frame::fields::{TschLinkOption, TschTimeslotTimings},
        mlme::tsch::{
            setlink::SetLinkRequest, setslotframe::SetSlotframeRequest, TschScheduleOperation,
        },
        primitives::{
            MacConfirm, MacRequest, ScanConfirm, ScanRequest, ScanType, MAX_PAN_DESCRIPTORS,
        },
        procedures::{join_network_from_scan_with, tsch_start_pan_configured, JoinPolicy},
        MacBufferAllocator, MacRequestSender,
    },
    scheduler::{command::tsch::TschRoleConfig, scan::ScanChannels, tsch::TschLinkType},
};

use crate::{val, Config};

/// Slotframe size of `minimal`, in timeslots: the minimal 6TiSCH schedule of
/// RFC 8180, one shared cell in a slotframe of 11.
const MINIMAL_SLOTFRAME_SIZE: u16 = 11;
/// Slotframe size of `chain`. Two timeslots per node (advertising and
/// up-traffic), so a chain of up to 15 nodes fits.
const CHAIN_SLOTFRAME_SIZE: u16 = 101;
/// Slotframe size of `star`.
const STAR_SLOTFRAME_SIZE: u16 = 101;
/// Slotframe size of `contention`. Kept short so the single shared up cell
/// recurs often and collisions are frequent.
const CONTENTION_SLOTFRAME_SIZE: u16 = 8;
/// Slotframe size of `mesh`.
const MESH_SLOTFRAME_SIZE: u16 = 100;
/// Timeslot spacing of `mesh`: node `i` transmits in timeslot
/// `i * MESH_SLOT_STRIDE`. A stride of 1 means `timeslot = NODE_ID`; 2 leaves
/// one idle slot between active ones, the density of the other layouts.
const MESH_SLOT_STRIDE: u16 = 1;
/// How many children the `star` root pre-installs a listen cell for, when
/// `NODE_COUNT` does not say. Child `k` transmits in timeslot `k`, so this
/// bounds `NODE_ID`. Each cell costs one link, so a root that installs them
/// all needs `DOT15D4_MAC_TSCH_MAX_LINKS` raised; `NODE_COUNT` avoids that.
const MAX_STAR_CHILDREN: u16 = 10;

/// Where this node's cells come from, selected by `SCHEDULE`.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Schedule {
    /// The minimal 6TiSCH schedule: one shared cell for everything. See
    /// [`install_minimal`].
    Minimal,
    /// Linear chain, parent = `NODE_ID - 1` (the default). See
    /// [`install_chain`].
    Chain,
    /// Every node above 0 is a child of the time root, each with its own up
    /// cell, so children never contend. See [`install_star`].
    Star,
    /// Every child shares one up cell with the root, so uplink transmissions
    /// collide and the shared-link CSMA-CA backoff runs. See
    /// [`install_contention`].
    Contention,
    /// Every node transmits in one cell of its own and listens in every other
    /// node's, so the cells impose no topology. See [`install_mesh`].
    Mesh,
    /// No cells of its own: adopt the slotframe and the links advertised in
    /// the beacon it joins through. Always a joining node, never the time
    /// root. See [`join`].
    Learn,
}

impl Schedule {
    fn from_env() -> Self {
        match option_env!("SCHEDULE") {
            None | Some("chain") => Schedule::Chain,
            Some("minimal") => Schedule::Minimal,
            Some("star") => Schedule::Star,
            Some("contention") => Schedule::Contention,
            Some("mesh") => Schedule::Mesh,
            Some("learn") => Schedule::Learn,
            Some(_) => {
                panic!("SCHEDULE must be one of: minimal, chain, star, contention, mesh, learn")
            }
        }
    }
}

/// `BEACON_PERIOD_S`: Enhanced Beacon period of every beaconing node, in
/// seconds.
fn beacon_period_s() -> u32 {
    option_env!("BEACON_PERIOD_S")
        .map(|s| s.parse().expect("BEACON_PERIOD_S must be an integer"))
        .unwrap_or(10)
}

/// `BEACON_JITTER_MS`: random `+/-` jitter on that period. Without it, two
/// neighbouring nodes that beacon on the same cadence in the same shared cell
/// can collide round after round.
fn beacon_jitter_ms() -> u16 {
    option_env!("BEACON_JITTER_MS")
        .map(|s| s.parse().expect("BEACON_JITTER_MS must be an integer"))
        .unwrap_or(0)
}

fn channel() -> Option<Channel> {
    option_env!("CHANNEL").map(|s| {
        let number: u8 = s.parse().expect("CHANNEL must be 11..=26");
        Channel::try_from(number).expect("CHANNEL must be 11..=26")
    })
}

/// `NODE_COUNT`: how many nodes take part. `mesh` needs it (it says how many
/// timeslots to listen in); `star` uses it to size the root's listen cells.
fn node_count() -> Option<u16> {
    option_env!("NODE_COUNT").map(|s| s.parse().expect("NODE_COUNT must be a small integer"))
}

fn slotframe_size(default: u16) -> u16 {
    option_env!("SLOTFRAME_SIZE")
        .map(|s| s.parse().expect("SLOTFRAME_SIZE must be an integer"))
        .unwrap_or(default)
}

/// `PARENT_ID`: the parent's `NODE_ID`. Optional and unused by the cell
/// layout; it only guards the id assignment. See [`install_mesh`].
fn parent_id() -> Option<u16> {
    option_env!("PARENT_ID").map(|s| s.parse().expect("PARENT_ID must be a small integer"))
}

/// Whether this node starts the PAN. A learning node has no schedule to
/// advertise before it joins, so it is never the time root, whatever `NODE_ID`
/// says.
pub(crate) fn is_time_root(config: &Config) -> bool {
    config.node_id == 0 && Schedule::from_env() != Schedule::Learn
}

pub async fn start(
    request_sender: &MacRequestSender<'static>,
    timer: NrfRadioSleepTimer,
    buffer_allocator: MacBufferAllocator,
    config: &Config,
) {
    let schedule = Schedule::from_env();
    install(schedule, request_sender, config).await;
    join(schedule, request_sender, timer, buffer_allocator, config).await;
}

pub async fn rejoin(
    request_sender: &MacRequestSender<'static>,
    timer: NrfRadioSleepTimer,
    buffer_allocator: MacBufferAllocator,
    config: &Config,
) {
    let schedule = Schedule::from_env();
    join(schedule, request_sender, timer, buffer_allocator, config).await;
}

/// Install the cells of the selected layout. `learn` installs none: they come
/// from the beacon, in [`join`].
async fn install(schedule: Schedule, request_sender: &MacRequestSender<'static>, config: &Config) {
    match schedule {
        Schedule::Minimal => install_minimal(request_sender).await,
        Schedule::Chain => install_chain(request_sender, config).await,
        Schedule::Star => install_star(request_sender, config).await,
        Schedule::Contention => install_contention(request_sender, config).await,
        Schedule::Mesh => install_mesh(request_sender, config).await,
        Schedule::Learn => {}
    }
}

async fn join(
    schedule: Schedule,
    request_sender: &MacRequestSender<'static>,
    timer: NrfRadioSleepTimer,
    buffer_allocator: MacBufferAllocator,
    config: &Config,
) {
    // The time root always beacons; elsewhere BEACON=0 turns Enhanced Beacons
    // off.
    let role = if is_time_root(config) {
        TschRoleConfig::time_root(beacon_period_s())
    } else if config.beacon {
        TschRoleConfig::device()
            .with_beacon(beacon_period_s())
            .with_beacon_jitter_ms(beacon_jitter_ms())
    } else {
        TschRoleConfig::device()
    };

    if is_time_root(config) {
        // The root defines the network timing: template 1, the default IEEE
        // 802.15.4 timeslot template, and the hopping sequence.
        let timings = TschTimeslotTimings::new(1, TschTimeslotTimings::DEFAULT_GUARD_TIME);
        tsch_start_pan_configured(
            request_sender,
            timer,
            role,
            Some(timings),
            hopping_sequence(),
        )
        .await;
        val::pan_started(val::uptime_ms(&timer));
        return;
    }

    assert!(
        config.parent.is_some() || schedule == Schedule::Learn,
        "a static layout needs PARENT: its cells point at the parent"
    );
    let policy = JoinPolicy {
        parent: config.parent,
        apply_advertised_schedule: schedule == Schedule::Learn,
    };

    // Scan until an acceptable beacon arrives, then join through its sender.
    loop {
        let scan_confirm = passive_scan(request_sender).await;
        val::scan_done(scan_confirm.pan_descriptor_list.len(), val::uptime_ms(&timer));

        if let Some((pan_id, parent)) = join_network_from_scan_with::<NrfRadioDriver>(
            request_sender,
            scan_confirm,
            buffer_allocator,
            role,
            policy,
        )
        .await
        {
            val::join_ok(&parent, pan_id, val::uptime_ms(&timer));
            #[cfg(feature = "tsch-association")]
            associate(request_sender, timer, pan_id, parent).await;
            return;
        }
        val::join_retry(val::uptime_ms(&timer));
    }
}

/// One passive scan on the network's channel.
async fn passive_scan(request_sender: &MacRequestSender<'static>) -> ScanConfirm {
    let request_token = request_sender.allocate_request_token().await;
    let mac_request = MacRequest::MlmeScan(ScanRequest {
        scan_type: ScanType::Passive,
        scan_channels: ScanChannels::Single(channel().unwrap_or(Channel::_26)),
        // The scan returns as soon as it holds `max_pan_descriptors` beacons,
        // so this exponent only bounds a scan that hears nobody.
        scan_duration: 10,
        max_pan_descriptors: MAX_PAN_DESCRIPTORS,
    });
    match request_sender
        .send_request_awaiting_response(request_token, mac_request)
        .await
    {
        MacConfirm::MlmeScan(scan_confirm) => scan_confirm,
        _ => unreachable!(),
    }
}

/// Run the MLME association handshake and read back the result.
///
/// The handshake writes macShortAddress on success, so reading the attribute
/// back is what tells the application, and the log, whether the coordinator
/// accepted the node. `0xffff` is the value the PIB starts from.
#[cfg(feature = "tsch-association")]
async fn associate(
    request_sender: &MacRequestSender<'static>,
    timer: NrfRadioSleepTimer,
    pan_id: u16,
    coordinator: [u8; 8],
) {
    use dot15d4::mac::{
        mlme::get::{GetConfirm, GetRequest, GetRequestAttribute},
        procedures::tsch_associate,
    };

    tsch_associate(request_sender, pan_id, coordinator).await;

    let request_token = request_sender.allocate_request_token().await;
    let mac_request = MacRequest::MlmeGet(GetRequest::new(GetRequestAttribute::MacShortAddress));
    let short_address = match request_sender
        .send_request_awaiting_response(request_token, mac_request)
        .await
    {
        MacConfirm::MlmeGet(GetConfirm::MacShortAddress(address)) => address,
        _ => 0xffff,
    };
    val::assoc(short_address, short_address != 0xffff, val::uptime_ms(&timer));
}

/// The hopping sequence the time root advertises: one entry when `CHANNEL`
/// pins the network to a single channel, the stack default otherwise.
fn hopping_sequence() -> Option<heapless::Vec<Channel, MAC_HOPPING_SEQUENCE_MAX_LENGTH>> {
    let channel = channel()?;
    let mut sequence = heapless::Vec::new();
    let _ = sequence.push(channel);
    Some(sequence)
}

/// Add slotframe #0, the only slotframe used here. It is advertised in
/// Enhanced Beacons, so a joining node can copy it.
async fn add_slotframe(request_sender: &MacRequestSender<'static>, size: u16) {
    let request_token = request_sender.allocate_request_token().await;
    let mac_request = MacRequest::MlmeSetSlotframe(SetSlotframeRequest {
        handle: 0,
        operation: TschScheduleOperation::Add,
        size,
        advertise: true,
    });
    request_sender
        .send_request_awaiting_response(request_token, mac_request)
        .await;
}

/// Add one cell to slotframe #0. Every cell uses channel offset 0.
///
/// `advertise` puts the cell in this node's own beacons; only advertised cells
/// can be learned by a joining node.
async fn add_cell(
    request_sender: &MacRequestSender<'static>,
    timeslot: u16,
    link_options: TschLinkOption,
    link_type: TschLinkType,
    neighbor: Option<[u8; 8]>,
    advertise: bool,
) {
    let request_token = request_sender.allocate_request_token().await;
    let mac_request = MacRequest::MlmeSetLink(SetLinkRequest {
        slotframe_handle: 0,
        channel_offset: 0,
        timeslot,
        link_options,
        link_type,
        neighbor,
        advertise,
    });
    request_sender
        .send_request_awaiting_response(request_token, mac_request)
        .await;
}

/// Shared advertising cell: Enhanced Beacons out, plus a shared fallback that
/// carries unicast to and from any neighbour.
async fn advertising_cell(
    request_sender: &MacRequestSender<'static>,
    timeslot: u16,
    advertise: bool,
) {
    add_cell(
        request_sender,
        timeslot,
        TschLinkOption::Tx | TschLinkOption::Rx | TschLinkOption::Shared,
        TschLinkType::Advertising,
        None,
        advertise,
    )
    .await;
}

/// Dedicated transmit cell towards `neighbour`.
async fn tx_cell(
    request_sender: &MacRequestSender<'static>,
    timeslot: u16,
    neighbour: Option<[u8; 8]>,
) {
    add_cell(
        request_sender,
        timeslot,
        TschLinkOption::Tx,
        TschLinkType::Normal,
        neighbour,
        false,
    )
    .await;
}

/// Receive cell. `None` listens for anyone.
async fn rx_cell(
    request_sender: &MacRequestSender<'static>,
    timeslot: u16,
    neighbour: Option<[u8; 8]>,
) {
    add_cell(
        request_sender,
        timeslot,
        TschLinkOption::Rx,
        TschLinkType::Normal,
        neighbour,
        false,
    )
    .await;
}

/// Shared transmit/receive cell towards `neighbour`. Several senders may pick
/// it, so the shared-link CSMA-CA backoff applies.
async fn shared_cell(
    request_sender: &MacRequestSender<'static>,
    timeslot: u16,
    neighbour: Option<[u8; 8]>,
) {
    add_cell(
        request_sender,
        timeslot,
        TschLinkOption::Tx | TschLinkOption::Rx | TschLinkOption::Shared,
        TschLinkType::Normal,
        neighbour,
        false,
    )
    .await;
}

async fn install_minimal(request_sender: &MacRequestSender<'static>) {
    add_slotframe(request_sender, slotframe_size(MINIMAL_SLOTFRAME_SIZE)).await;
    advertising_cell(request_sender, 0, true).await;
}

async fn install_chain(request_sender: &MacRequestSender<'static>, config: &Config) {
    let id = config.node_id;
    add_slotframe(request_sender, slotframe_size(CHAIN_SLOTFRAME_SIZE)).await;

    if id == 0 || config.beacon {
        advertising_cell(request_sender, 2 * id, true).await;
    }
    if id > 0 {
        rx_cell(request_sender, 2 * (id - 1), config.parent).await;
        tx_cell(request_sender, 2 * id + 1, config.parent).await;
    }
    rx_cell(request_sender, 2 * id + 3, None).await;
}

async fn install_star(request_sender: &MacRequestSender<'static>, config: &Config) {
    let id = config.node_id;
    add_slotframe(request_sender, slotframe_size(STAR_SLOTFRAME_SIZE)).await;

    if id == 0 {
        let children = node_count().map(|n| n.saturating_sub(1)).unwrap_or(MAX_STAR_CHILDREN);
        advertising_cell(request_sender, 0, true).await;
        for child in 1..=children {
            rx_cell(request_sender, child, None).await;
        }
    } else {
        rx_cell(request_sender, 0, config.parent).await;
        tx_cell(request_sender, id, config.parent).await;
        if config.beacon {
            advertising_cell(request_sender, MAX_STAR_CHILDREN + id, true).await;
        }
    }
}

async fn install_contention(request_sender: &MacRequestSender<'static>, config: &Config) {
    add_slotframe(request_sender, slotframe_size(CONTENTION_SLOTFRAME_SIZE)).await;

    if config.node_id == 0 {
        advertising_cell(request_sender, 0, true).await;
        rx_cell(request_sender, 1, None).await;
    } else {
        rx_cell(request_sender, 0, config.parent).await;
        shared_cell(request_sender, 1, config.parent).await;
    }
}

async fn install_mesh(request_sender: &MacRequestSender<'static>, config: &Config) {
    let id = config.node_id;
    let node_count = node_count().expect("SCHEDULE=mesh needs NODE_COUNT");
    assert!(
        id < node_count,
        "SCHEDULE=mesh: NODE_ID must be below NODE_COUNT"
    );
    if let Some(parent_id) = parent_id() {
        assert!(
            parent_id < id,
            "SCHEDULE=mesh: PARENT_ID must be below NODE_ID, so that the \
             parent's advertising cell comes first in the slotframe and this \
             node resynchronizes before it beacons"
        );
    }

    add_slotframe(request_sender, slotframe_size(MESH_SLOTFRAME_SIZE)).await;

    for i in 0..node_count {
        let timeslot = i * MESH_SLOT_STRIDE;
        if i == id {
            advertising_cell(request_sender, timeslot, false).await;
        } else {
            rx_cell(request_sender, timeslot, None).await;
        }
    }
}
