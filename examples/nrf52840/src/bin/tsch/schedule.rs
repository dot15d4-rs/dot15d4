use dot15d4::{
    driver::socs::nrf::{NrfRadioDriver, NrfRadioSleepTimer},
    mac::{
        frame::fields::TschLinkOption,
        mlme::tsch::{
            setlink::SetLinkRequest, setslotframe::SetSlotframeRequest, TschScheduleOperation,
        },
        primitives::MacRequest,
        procedures::{
            join_network_from_scan_with, scan, tsch_start_pan_configured, JoinPolicy,
        },
        MacBufferAllocator, MacRequestSender,
    },
    scheduler::{command::tsch::TschRoleConfig, tsch::TschLinkType},
};

/// Enhanced Beacon period of every beaconing node, in seconds.
const BEACON_PERIOD_SECS: u32 = 9;
/// Random +/- jitter on the beacon period, in milliseconds.
const BEACON_JITTER_MS: u16 = 500;

/// Slotframe size of the chain schedule. Two timeslots per node (advertising
/// and up-traffic), so this fits a chain of up to 15 nodes.
const CHAIN_SLOTFRAME_SIZE: u16 = 100;
/// Slotframe size for the `star` schedule.
const STAR_SLOTFRAME_SIZE: u16 = 32;
/// Number of children the `star` root pre-installs a listen cell for. Child
/// `k` uses timeslot `k` as its dedicated up cell, so this bounds NODE_ID.
const MAX_STAR_CHILDREN: u16 = 8;
/// Slotframe size for the `contention` schedule. Kept small so the single
/// shared up cell recurs often and collisions are frequent.
const CONTENTION_SLOTFRAME_SIZE: u16 = 8;

/// Which experiment schedule to install, selected by the `SCHEDULE`
/// compile-time environment variable.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Schedule {
    /// Linear chain, parent = NODE_ID - 1 (the default). See [`install_chain`].
    Chain,
    /// Star: every NODE_ID > 0 is a child of the time root (NODE_ID 0), each on
    /// its own dedicated up cell (timeslot = NODE_ID). No uplink contention.
    Star,
    /// Contention star: every child shares one common shared up cell (timeslot
    /// 1) with the root, so their uplink transmissions collide and exercise the
    /// TSCH shared-link CSMA-CA backoff.
    Contention,
}

impl Schedule {
    fn from_env() -> Self {
        match option_env!("SCHEDULE") {
            None | Some("chain") => Schedule::Chain,
            Some("star") => Schedule::Star,
            Some("contention") => Schedule::Contention,
            Some(_) => panic!("SCHEDULE must be one of: chain, star, contention"),
        }
    }
}

/// Install this node's cells for the schedule selected by `SCHEDULE`.
pub async fn install(
    request_sender: &MacRequestSender<'static>,
    node_id: u16,
    parent: Option<[u8; 8]>,
    beacon: bool,
) {
    match Schedule::from_env() {
        Schedule::Chain => install_chain(request_sender, node_id, parent, beacon).await,
        Schedule::Star => install_star(request_sender, node_id, parent, beacon).await,
        Schedule::Contention => install_contention(request_sender, node_id, parent).await,
    }
}

pub async fn start_or_join(
    request_sender: &MacRequestSender<'static>,
    timer: NrfRadioSleepTimer,
    buffer_allocator: MacBufferAllocator,
    node_id: u16,
    parent: Option<[u8; 8]>,
    beacon: bool,
) {
    // The time root always beacons. On other nodes, BEACON=0 disables
    // Enhanced Beacon transmission (default: every node beacons). The jitter
    // keeps equal-period beacons of neighbouring nodes from colliding.
    let role = if node_id == 0 {
        TschRoleConfig::time_root(BEACON_PERIOD_SECS)
    } else if beacon {
        TschRoleConfig::device()
            .with_beacon(BEACON_PERIOD_SECS)
            .with_beacon_jitter_ms(BEACON_JITTER_MS)
    } else {
        TschRoleConfig::device()
    };

    if node_id == 0 {
        tsch_start_pan_configured(request_sender, timer, role, None, None).await;
        return;
    }

    let parent = parent.expect("NODE_ID > 0 needs the PARENT env variable");
    let policy = JoinPolicy {
        parent: Some(parent),
        apply_advertised_schedule: false,
    };
    // Scan until the parent is heard, then join through it.
    loop {
        let scan_confirm = scan(request_sender).await;
        if let Some(_pan) = join_network_from_scan_with::<NrfRadioDriver>(
            request_sender,
            scan_confirm,
            buffer_allocator,
            role,
            policy,
        )
        .await
        {
            #[cfg(feature = "tsch-association")]
            {
                use dot15d4::mac::procedures::tsch_associate;

                let (pan_id, coord_address) = _pan;
                tsch_associate(request_sender, pan_id, coord_address).await;
            }
            break;
        }
    }
}

/// Add slotframe #0 (advertised in Enhanced Beacons) of the given size.
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

/// Add one link to slotframe #0.
async fn add_link(
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

async fn install_chain(
    request_sender: &MacRequestSender<'static>,
    node_id: u16,
    parent: Option<[u8; 8]>,
    beacon: bool,
) {
    add_slotframe(request_sender, CHAIN_SLOTFRAME_SIZE).await;

    // This node's own shared advertising cell (root always, others only when
    // they actually send beacons).
    if node_id == 0 || beacon {
        add_link(
            request_sender,
            2 * node_id,
            TschLinkOption::Tx | TschLinkOption::Rx | TschLinkOption::Shared,
            TschLinkType::Advertising,
            None,
            true,
        )
        .await;
    }

    if node_id > 0 {
        add_link(
            request_sender,
            2 * (node_id - 1),
            TschLinkOption::Rx,
            TschLinkType::Normal,
            parent,
            false,
        )
        .await;
        // Dedicated up cell to the parent.
        add_link(
            request_sender,
            2 * node_id + 1,
            TschLinkOption::Tx,
            TschLinkType::Normal,
            parent,
            false,
        )
        .await;
    }

    add_link(
        request_sender,
        2 * node_id + 3,
        TschLinkOption::Rx,
        TschLinkType::Normal,
        None,
        false,
    )
    .await;
}

async fn install_star(
    request_sender: &MacRequestSender<'static>,
    node_id: u16,
    parent: Option<[u8; 8]>,
    beacon: bool,
) {
    add_slotframe(request_sender, STAR_SLOTFRAME_SIZE).await;

    if node_id == 0 {
        add_link(
            request_sender,
            0,
            TschLinkOption::Tx | TschLinkOption::Rx | TschLinkOption::Shared,
            TschLinkType::Advertising,
            None,
            true,
        )
        .await;
        for child in 1..=MAX_STAR_CHILDREN {
            add_link(
                request_sender,
                child,
                TschLinkOption::Rx,
                TschLinkType::Normal,
                None,
                false,
            )
            .await;
        }
    } else {
        add_link(
            request_sender,
            0,
            TschLinkOption::Rx,
            TschLinkType::Normal,
            parent,
            false,
        )
        .await;
        add_link(
            request_sender,
            node_id,
            TschLinkOption::Tx,
            TschLinkType::Normal,
            parent,
            false,
        )
        .await;
        if beacon {
            add_link(
                request_sender,
                MAX_STAR_CHILDREN + node_id,
                TschLinkOption::Tx | TschLinkOption::Rx | TschLinkOption::Shared,
                TschLinkType::Advertising,
                None,
                true,
            )
            .await;
        }
    }
}

async fn install_contention(
    request_sender: &MacRequestSender<'static>,
    node_id: u16,
    parent: Option<[u8; 8]>,
) {
    add_slotframe(request_sender, CONTENTION_SLOTFRAME_SIZE).await;

    if node_id == 0 {
        add_link(
            request_sender,
            0,
            TschLinkOption::Tx | TschLinkOption::Rx | TschLinkOption::Shared,
            TschLinkType::Advertising,
            None,
            true,
        )
        .await;
        add_link(
            request_sender,
            1,
            TschLinkOption::Rx,
            TschLinkType::Normal,
            None,
            false,
        )
        .await;
    } else {
        add_link(
            request_sender,
            0,
            TschLinkOption::Rx,
            TschLinkType::Normal,
            parent,
            false,
        )
        .await;
        add_link(
            request_sender,
            1,
            TschLinkOption::Tx | TschLinkOption::Rx | TschLinkOption::Shared,
            TschLinkType::Normal,
            parent,
            false,
        )
        .await;
    }
}
