use core::marker::PhantomData;

use dot15d4_driver::radio::frame::{
    Address, AddressingMode, AddressingRepr, FrameType, FrameVersion, IeListRepr, IeRepr,
    IeReprList, PanId, PanIdCompressionRepr, RadioFrame, RadioFrameSized, RadioFrameUnsized,
};
use dot15d4_driver::radio::DriverConfig;
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_frame::repr::{MpduRepr, SeqNrRepr};
use dot15d4_frame::{MpduWithAllFields, MpduWithIes};
use dot15d4_util::allocator::{BufferToken, IntoBuffer};

use crate::scheduler::SchedulerService;

use super::schedule::{TschAsn, TschPib};

/// Static storage for Enhanced Beacon IE configuration
/// This is used because MpduRepr requires 'static lifetimes for const IE lists
pub struct EnhancedBeaconBuilder<'buffer, RadioDriverImpl: DriverConfig> {
    mpdu_repr: MpduRepr<'buffer, MpduWithIes>,
    _phantom: PhantomData<RadioDriverImpl>,
}

impl<'buffer, RadioDriverImpl: DriverConfig> EnhancedBeaconBuilder<'buffer, RadioDriverImpl> {
    pub const fn new() -> Self {
        // Build the MPDU representation
        static SLOTFRAMES: [u8; 1] = [1]; // 1 slotframe with 1 link
        static IES: [IeRepr; 3] = [
            IeRepr::TschSynchronizationNestedIe,
            IeRepr::FullTschTimeslotNestedIe,
            IeRepr::TschSlotframeAndLinkNestedIe(&SLOTFRAMES),
        ];
        static IE_REPR_LIST: IeReprList<'static, IeRepr> = IeReprList::new(&IES);
        static IE_LIST: IeListRepr<'static> = IeListRepr::WithoutTerminationIes(IE_REPR_LIST);

        const MPDU_REPR: MpduRepr<'_, MpduWithIes> = MpduRepr::new()
            .with_frame_control(SeqNrRepr::No)
            .with_addressing(AddressingRepr::new(
                AddressingMode::Short,
                AddressingMode::Extended,
                true,
                PanIdCompressionRepr::Yes,
            ))
            .without_security()
            .with_ies(IE_LIST);
        Self {
            mpdu_repr: MPDU_REPR,
            _phantom: PhantomData,
        }
    }

    /// Build an Enhanced Beacon frame from the schedule
    pub fn build_enhanced_beacon(
        &self,
        service: &SchedulerService<RadioDriverImpl>,
        radio_frame: RadioFrame<RadioFrameUnsized>,
        src_address: &Address<[u8; 8]>,
        pan_id: &PanId<[u8; 2]>,
    ) -> Option<RadioFrame<RadioFrameSized>> {
        let buffer = radio_frame.into_buffer();
        // Create the frame writer
        let mut mpdu_writer = self
            .mpdu_repr
            .into_writer::<RadioDriverImpl>(FrameVersion::Ieee802154, FrameType::Beacon, 0, buffer)
            .ok()?;

        // Set addressing fields
        {
            let mut addressing = mpdu_writer.addressing_fields_mut();
            addressing.src_address_mut().set(src_address);
            addressing
                .dst_address_mut()
                .set(&Address::<&[u8]>::BROADCAST_ADDR);
            addressing.dst_pan_id_mut().set(pan_id);
        }

        // Set IE content from schedule
        {
            let mut ies = mpdu_writer.ies_fields_mut();

            // Set Slotframe and Link IE from schedule
            if let Some(mut slotframe_ie) = ies.tsch_slotframe_link_mut() {
                let mut slotframes = slotframe_ie.slotframes_mut();

                // Iterate through schedule's slotframes
                for (sf_handle, sf_size) in service.slotframe_info() {
                    if let Some(mut sf) = slotframes.next() {
                        sf.set_handle(sf_handle as u8);
                        sf.set_size(sf_size);

                        // Add links for this slotframe
                        let mut links = sf.links_mut();
                        for link in service
                            .links()
                            .filter(|l| l.slotframe_handle == sf_handle && l.link_advertise)
                        {
                            if let Some(mut l) = links.next() {
                                l.set_timeslot(link.timeslot);
                                l.set_channel_offset(link.channel_offset);
                                l.set_options(link.link_options.bits());
                            }
                        }
                    }
                }
            }

            // Set Timeslot IE from schedule's timing configuration
            if let Some(mut timeslot_ie) = ies.tsch_timeslot_mut() {
                let timings = &service.pib.tsch.timeslot_timings;
                timeslot_ie.set_timeslot_id(timings.id());
                timeslot_ie.set_cca_offset(timings.cca_offset());
                timeslot_ie.set_cca(timings.cca());
                timeslot_ie.set_tx_offset(timings.tx_offset());
                timeslot_ie.set_rx_offset(timings.rx_offset());
                timeslot_ie.set_rx_ack_delay(timings.rx_ack_delay());
                timeslot_ie.set_tx_ack_delay(timings.tx_ack_delay());
                timeslot_ie.set_rx_wait(timings.rx_wait());
                timeslot_ie.set_ack_wait(timings.ack_wait());
                timeslot_ie.set_rx_tx(timings.rx_tx());
                timeslot_ie.set_max_ack(timings.max_ack());
                timeslot_ie.set_max_tx(timings.max_tx());
                timeslot_ie.set_timeslot_length(timings.timeslot_length());
            }
        }

        Some(mpdu_writer.into_radio_frame::<RadioDriverImpl>())
    }

    pub fn update_beacon(
        &self,
        radio_frame: RadioFrame<RadioFrameSized>,
        asn: TschAsn,
    ) -> Option<RadioFrame<RadioFrameSized>> {
        let buffer = radio_frame.into_buffer();
        let mut mpdu_writer = self
            .mpdu_repr
            .into_writer::<RadioDriverImpl>(FrameVersion::Ieee802154, FrameType::Beacon, 0, buffer)
            .ok()?;

        let mut ies = mpdu_writer.ies_fields_mut();
        // Update TSCH Synchronization IE
        let mut tsch_sync = ies.tsch_sync_mut().unwrap();
        tsch_sync.set_asn(asn);
        //TODO: support join metric
        tsch_sync.set_join_metric(0);

        Some(mpdu_writer.into_radio_frame::<RadioDriverImpl>())
    }
}
