#![allow(dead_code)]

use dot15d4::driver::{socs::nrf::NrfRadioSleepTimer, timer::RadioTimerApi};

/// Milliseconds since boot, the `ms` field of every record.
pub fn uptime_ms(timer: &NrfRadioSleepTimer) -> u64 {
    timer.now().duration_since_epoch().to_millis()
}

pub fn boot(node_id: u16, role: &'static str, address: &[u8; 8], ms: u64) {
    #[cfg(any(feature = "log", feature = "defmt"))]
    dot15d4::util::info!(
        "val boot node={} role={} addr={:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x} ms={}",
        node_id,
        role,
        address[0],
        address[1],
        address[2],
        address[3],
        address[4],
        address[5],
        address[6],
        address[7],
        ms
    );
    #[cfg(not(any(feature = "log", feature = "defmt")))]
    let _ = (node_id, role, address, ms);
}

/// The time root has started the PAN and is beaconing.
pub fn pan_started(ms: u64) {
    #[cfg(any(feature = "log", feature = "defmt"))]
    dot15d4::util::info!("val pan started ms={}", ms);
    #[cfg(not(any(feature = "log", feature = "defmt")))]
    let _ = ms;
}

/// One passive scan finished, with the number of beacons it collected.
pub fn scan_done(found: usize, ms: u64) {
    #[cfg(any(feature = "log", feature = "defmt"))]
    dot15d4::util::info!("val scan done found={} ms={}", found, ms);
    #[cfg(not(any(feature = "log", feature = "defmt")))]
    let _ = (found, ms);
}

/// Joined: the node has taken over the network's timing and is synchronized.
pub fn join_ok(parent: &[u8; 8], pan_id: u16, ms: u64) {
    #[cfg(any(feature = "log", feature = "defmt"))]
    dot15d4::util::info!(
        "val join ok parent={:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x} pan={:04x} ms={}",
        parent[0],
        parent[1],
        parent[2],
        parent[3],
        parent[4],
        parent[5],
        parent[6],
        parent[7],
        pan_id,
        ms
    );
    #[cfg(not(any(feature = "log", feature = "defmt")))]
    let _ = (parent, pan_id, ms);
}

/// No acceptable beacon in this scan; scanning again.
pub fn join_retry(ms: u64) {
    #[cfg(any(feature = "log", feature = "defmt"))]
    dot15d4::util::info!("val join retry ms={}", ms);
    #[cfg(not(any(feature = "log", feature = "defmt")))]
    let _ = ms;
}

/// The node lost synchronization and left the network; it will scan and join
/// again.
pub fn desync(ms: u64) {
    #[cfg(any(feature = "log", feature = "defmt"))]
    dot15d4::util::info!("val desync ms={}", ms);
    #[cfg(not(any(feature = "log", feature = "defmt")))]
    let _ = ms;
}

/// Association handshake outcome, with the short address the PIB holds after
/// it.
pub fn assoc(short_address: u16, ok: bool, ms: u64) {
    #[cfg(any(feature = "log", feature = "defmt"))]
    if ok {
        dot15d4::util::info!("val assoc ok short={:04x} ms={}", short_address, ms);
    } else {
        dot15d4::util::info!("val assoc fail ms={}", ms);
    }
    #[cfg(not(any(feature = "log", feature = "defmt")))]
    let _ = (short_address, ok, ms);
}

/// A child associated with this coordinator, and the short address it was
/// given.
pub fn assoc_peer(device: &[u8; 8], short_address: u16, ms: u64) {
    #[cfg(any(feature = "log", feature = "defmt"))]
    dot15d4::util::info!(
        "val assoc peer dev={:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x} short={:04x} ms={}",
        device[0],
        device[1],
        device[2],
        device[3],
        device[4],
        device[5],
        device[6],
        device[7],
        short_address,
        ms
    );
    #[cfg(not(any(feature = "log", feature = "defmt")))]
    let _ = (device, short_address, ms);
}

/// One data frame handed to the MAC. `ok` means its acknowledgment arrived.
pub fn tx(sequence: u32, ok: bool, ms: u64) {
    #[cfg(any(feature = "log", feature = "defmt"))]
    dot15d4::util::info!(
        "val tx seq={} ok={} ms={}",
        sequence,
        if ok { 1u8 } else { 0u8 },
        ms
    );
    #[cfg(not(any(feature = "log", feature = "defmt")))]
    let _ = (sequence, ok, ms);
}

/// One data frame delivered to this node.
pub fn rx(sequence: u32, source: &[u8; 8], ms: u64) {
    #[cfg(any(feature = "log", feature = "defmt"))]
    dot15d4::util::info!(
        "val rx seq={} src={:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x} ms={}",
        sequence,
        source[0],
        source[1],
        source[2],
        source[3],
        source[4],
        source[5],
        source[6],
        source[7],
        ms
    );
    #[cfg(not(any(feature = "log", feature = "defmt")))]
    let _ = (sequence, source, ms);
}
