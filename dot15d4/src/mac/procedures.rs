use dot15d4_driver::radio::frame::Address;
#[cfg(feature = "tsch")]
use dot15d4_driver::{
    radio::DriverConfig,
    timer::{NsInstant, RadioTimerApi},
};
#[cfg(feature = "tsch")]
use dot15d4_frame::{fields::MpduParser, mpdu::MpduFrame, MpduWithAllFields};

use crate::mac::{
    mlme::get::{GetRequest, GetRequestAttribute},
    primitives::MacRequest,
};

#[cfg(feature = "tsch")]
use super::{primitives::ScanConfirm, MacBufferAllocator, MacRequestSender};
#[cfg(feature = "tsch")]
use dot15d4_util::allocator::IntoBuffer;

pub async fn get_device_extended_address(
    request_sender: &MacRequestSender<'static>,
) -> Address<[u8; 8]> {
    let request_token = request_sender.allocate_request_token().await;
    let mac_request = MacRequest::MlmeGet(GetRequest::new(GetRequestAttribute::MacExtendedAddress));

    let confirm = request_sender
        .send_request_awaiting_response(request_token, mac_request)
        .await;

    match confirm {
        super::primitives::MacConfirm::MlmeGet(
            super::mlme::get::GetConfirm::MacExtendedAddress(addr),
        ) => addr,
        _ => unreachable!(),
    }
}

pub async fn get_coordinator_extended_address(
    request_sender: &MacRequestSender<'static>,
) -> Address<[u8; 8]> {
    let request_token = request_sender.allocate_request_token().await;
    let mac_request = MacRequest::MlmeGet(GetRequest::new(
        GetRequestAttribute::MacCoordExtendedAddress,
    ));
    let confirm = request_sender
        .send_request_awaiting_response(request_token, mac_request)
        .await;
    match confirm {
        super::primitives::MacConfirm::MlmeGet(
            super::mlme::get::GetConfirm::MacCoordExtendedAddress(addr),
        ) => addr,
        _ => unreachable!(),
    }
}

#[cfg(feature = "tsch")]
pub async fn tsch_start_pan<RadioTimerImpl: RadioTimerApi>(
    request_sender: &MacRequestSender<'static>,
    timer: RadioTimerImpl,
) {
    use crate::scheduler::tsch::TschLinkType;

    use super::{
        frame::fields::TschLinkOption,
        mlme::{
            set::{SetRequest, SetRequestAttribute},
            tsch::{
                setlink::SetLinkRequest, setslotframe::SetSlotframeRequest, TschScheduleOperation,
            },
        },
        primitives::{MacRequest, TschModeRequest},
    };

    // We create a single slotframe with handle 0 and size 100
    let request_token = request_sender.allocate_request_token().await;
    let mac_request = MacRequest::MlmeSetSlotframe(SetSlotframeRequest {
        handle: 0,                             // Slotframe Identifier
        operation: TschScheduleOperation::Add, // we want to add a slotframe
        size: 100,                             // Size of the sloframe in timeslots
        advertise: true, // The slotframe will be advertised in Enhanced Beacons
    });
    request_sender
        .send_request_awaiting_response(request_token, mac_request)
        .await;

    // Then, we add a link (for advertising) to that new slotframe
    let request_token = request_sender.allocate_request_token().await;
    let mac_request = MacRequest::MlmeSetLink(SetLinkRequest {
        slotframe_handle: 0, // handle of the associated slotframe
        channel_offset: 0,   // Channel offset used for the link
        timeslot: 0,         // Timeslot to use in the slotframe
        link_options: TschLinkOption::Tx
            | TschLinkOption::Rx
            | TschLinkOption::Shared
            | TschLinkOption::TimeKeeping, // Link used only for transmissions
        link_type: TschLinkType::Advertising, // Used for data transmission and not for advertising
        neighbor: None,
        advertise: true, // The link will be advertised in Enhanced Beacons
    });

    // We submit the request to the MAC service and wait for the operation to be completed
    request_sender
        .send_request_awaiting_response(request_token, mac_request)
        .await;

    // Then, we configure absolute timing (i.e. t0 for ASN 0)
    let start_timestamp = timer.now();
    let request_token = request_sender.allocate_request_token().await;
    let mac_request = MacRequest::MlmeSet(SetRequest::new(SetRequestAttribute::MacAsn(
        0,
        start_timestamp,
    )));
    request_sender
        .send_request_awaiting_response(request_token, mac_request)
        .await;

    // Finally, we switch to TSCH mode
    let request_token = request_sender.allocate_request_token().await;
    let mac_request = MacRequest::MlmeTschMode(TschModeRequest {
        tsch_mode: true,
        tsch_cca: false,
    });
    request_sender
        .send_request_awaiting_response(request_token, mac_request)
        .await;
    // info!("TSCH PAN Started");
}

#[cfg(feature = "tsch")]
pub async fn join_network_from_scan<RadioDriverImpl: DriverConfig>(
    request_sender: &MacRequestSender<'static>,
    scan_confirm: ScanConfirm,
    buffer_allocator: MacBufferAllocator,
) -> Option<(u16, [u8; 8])> {
    let mut best_candidate = None;
    for descriptor in scan_confirm.pan_descriptor_list {
        let timestamp = descriptor.timestamp;
        let parser = descriptor.mpdu.into_parser().parse_addressing().unwrap();
        let parser = parser
            .parse_security()
            .parse_ies::<RadioDriverImpl>()
            .unwrap();
        let ies = parser.ies_fields();
        let join_metric = ies.tsch_sync().map(|ts| ts.join_metric());

        if let Some(join_metric) = join_metric {
            if let Some((best_join_metric, best_mpdu, best_timestamp)) = best_candidate {
                if join_metric < best_join_metric {
                    best_candidate = Some((join_metric, parser, timestamp));
                    unsafe { buffer_allocator.deallocate_buffer(best_mpdu.into_buffer()) };
                } else {
                    best_candidate = Some((best_join_metric, best_mpdu, best_timestamp));
                    unsafe { buffer_allocator.deallocate_buffer(parser.into_buffer()) };
                }
            } else {
                best_candidate = Some((join_metric, parser, timestamp));
            }
        } else {
            unsafe { buffer_allocator.deallocate_buffer(parser.into_buffer()) };
        }
    }

    if let Some((_, mpdu_parser, timestamp)) = best_candidate {
        join_network_from_beacon(request_sender, mpdu_parser, timestamp, buffer_allocator).await
    } else {
        None
    }
}

#[cfg(feature = "tsch")]
async fn join_network_from_beacon(
    request_sender: &MacRequestSender<'static>,
    mpdu_parser: MpduParser<MpduFrame, MpduWithAllFields>,
    rx_timestamp: NsInstant,
    buffer_allocator: MacBufferAllocator,
) -> Option<(u16, [u8; 8])> {
    use super::{
        frame::fields::TschLinkOption,
        mlme::{
            set::{SetRequest, SetRequestAttribute},
            tsch::{
                setlink::SetLinkRequest, setslotframe::SetSlotframeRequest, TschScheduleOperation,
            },
        },
    };
    use crate::scheduler::tsch::TschLinkType;

    let ies = mpdu_parser.ies_fields();

    let sf_links_ie = ies.tsch_slotframe_link();
    if let Some(sf_links_ie) = sf_links_ie {
        use dot15d4_driver::radio::frame::Address;

        use crate::mac::primitives::MacRequest;

        use super::primitives::TschModeRequest;

        for slotframe in sf_links_ie.slotframes() {
            // We add the slotframe to our schedule
            let request_token = request_sender.allocate_request_token().await;
            let mac_request = MacRequest::MlmeSetSlotframe(SetSlotframeRequest {
                handle: slotframe.handle(),            // Slotframe Identifier
                operation: TschScheduleOperation::Add, // we want to add a slotframe
                size: slotframe.size(),                // Size of the sloframe in timeslots
                advertise: true, // The slotframe will be advertised in Enhanced Beacons
            });
            // info!(
            //     "Add Slotframe #{} with size {} :",
            //     (slotframe.handle()),
            //     (slotframe.size())
            // );
            request_sender
                .send_request_awaiting_response(request_token, mac_request)
                .await;
            for link in slotframe.links() {
                let request_token = request_sender.allocate_request_token().await;
                let mac_request = MacRequest::MlmeSetLink(SetLinkRequest {
                    slotframe_handle: slotframe.handle(),
                    channel_offset: link.channel_offset(),
                    timeslot: link.timeslot(),
                    link_options: TschLinkOption::from_bits(link.options()).unwrap(),
                    link_type: TschLinkType::Advertising,
                    neighbor: None,
                    advertise: true,
                });
                // info!(
                //     " * Add Link at timeslot={}, channel_offset={} with options {:b}",
                //     (link.timeslot()),
                //     (link.channel_offset()),
                //     (link.options())
                // );
                request_sender
                    .send_request_awaiting_response(request_token, mac_request)
                    .await;
            }
        }

        let request_token = request_sender.allocate_request_token().await;
        let asn = ies.tsch_sync().unwrap().asn();
        let mac_request = MacRequest::MlmeSet(SetRequest::new(SetRequestAttribute::MacAsn(
            asn,
            rx_timestamp,
        )));
        request_sender
            .send_request_awaiting_response(request_token, mac_request)
            .await;
        // info!("ASN:{} rx_timestamp:{}", asn, (rx_timestamp.ticks()));

        // TODO: join metric

        // Finally, we switch to TSCH mode
        let request_token = request_sender.allocate_request_token().await;
        let mac_request = MacRequest::MlmeTschMode(TschModeRequest {
            tsch_mode: true,
            tsch_cca: false,
        });
        request_sender
            .send_request_awaiting_response(request_token, mac_request)
            .await;

        let addressing = mpdu_parser.try_addressing_fields().unwrap();
        let pan_id = addressing.try_dst_pan_id().unwrap().into_u16();
        let src_address = addressing.try_src_address().unwrap();
        let src_address = match src_address {
            Address::Extended(extended_address) => extended_address,
            _ => unreachable!(),
        };
        let mut coord_address = [0u8; 8];
        coord_address.copy_from_slice(src_address.as_ref());

        unsafe { buffer_allocator.deallocate_buffer(mpdu_parser.into_buffer()) };
        // info!("Synchronized to TSCH PAN");

        let request_token = request_sender.allocate_request_token().await;
        let mac_request = MacRequest::MlmeSet(SetRequest::new(
            SetRequestAttribute::MacCoordExtendedAddress(coord_address),
        ));

        let _ = request_sender
            .send_request_awaiting_response(request_token, mac_request)
            .await;

        Some((pan_id, coord_address))
    } else {
        None
    }
}

#[cfg(feature = "tsch")]
pub async fn scan(request_sender: &MacRequestSender<'static>) -> ScanConfirm {
    use crate::{
        driver::radio::config::Channel,
        mac::primitives::{MacConfirm, MacRequest, ScanRequest, ScanType},
        scheduler::scan::ScanChannels,
    };

    // Finally, we switch to TSCH mode
    let request_token = request_sender.allocate_request_token().await;
    let mac_request = MacRequest::MlmeScan(ScanRequest {
        scan_type: ScanType::Passive,
        scan_channels: ScanChannels::Single(Channel::_26),
        scan_duration: 12,
        max_pan_descriptors: 1,
    });
    let response = request_sender
        .send_request_awaiting_response(request_token, mac_request)
        .await;
    // info!("Scanned Channels");
    match response {
        MacConfirm::MlmeScan(scan_confirm) => scan_confirm,
        _ => unreachable!(),
    }
}

#[cfg(feature = "tsch")]
pub async fn tsch_associate(
    request_sender: &MacRequestSender<'static>,
    pan_id: u16,
    coord_address: [u8; 8],
) {
    use super::{
        frame::mpdu::{AssociationStatus, CapabilityInformation},
        mlme::set::{SetRequest, SetRequestAttribute},
        primitives::{AssociateConfirm, AssociateRequest, MacConfirm, MacRequest},
    };

    let request_token = request_sender.allocate_request_token().await;
    let mac_request = MacRequest::MlmeAssociate(AssociateRequest::new(
        coord_address,
        pan_id,
        CapabilityInformation::default(),
    ));
    let response = request_sender
        .send_request_awaiting_response(request_token, mac_request)
        .await;

    match response {
        MacConfirm::MlmeAssociate(AssociateConfirm::Completed {
            status,
            short_address,
        }) => {
            if status == AssociationStatus::Successful {
                let addr_bytes = short_address.as_ref();
                let short_addr_u16 = u16::from_le_bytes([addr_bytes[0], addr_bytes[1]]);
                // info!("Associated with short address: {:04X}", short_addr_u16);

                // MLME-SET short address from associate confirm.
                let request_token = request_sender.allocate_request_token().await;
                let mac_request = MacRequest::MlmeSet(SetRequest::new(
                    SetRequestAttribute::MacShortAddress(short_addr_u16),
                ));
                request_sender
                    .send_request_awaiting_response(request_token, mac_request)
                    .await;
                // info!("Short address configured");
            } else {
                // info!("Association failed");
            }
        }
        MacConfirm::MlmeAssociate(AssociateConfirm::NoAck) => {
            // info!("Association request not acknowledged");
        }
        MacConfirm::MlmeAssociate(AssociateConfirm::ChannelAccessFailure) => {
            // info!("Association failed: channel access failure");
        }
        _ => unreachable!(),
    }
}
