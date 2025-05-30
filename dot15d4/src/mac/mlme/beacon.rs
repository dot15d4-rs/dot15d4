#![allow(dead_code)]
use crate::mac::frame::mpdu::MpduFrame;

pub struct BeaconRequest {}

pub struct BeaconConfirm {}

pub struct BeaconNotifyIndication {
    /// The received beacon frame
    pub mpdu: MpduFrame,
    /// Timestamp of frame reception
    pub timestamp: u32,
}
