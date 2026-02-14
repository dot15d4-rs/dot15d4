//! Enhanced Beacon frame builder for TSCH.

use core::marker::PhantomData;

use dot15d4_driver::radio::{
    frame::{
        Address, AddressingMode, AddressingRepr, FrameType, FrameVersion, IeListRepr, IeRepr,
        IeReprList, PanIdCompressionRepr, RadioFrame, RadioFrameUnsized,
    },
    DriverConfig,
};
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_frame::repr::{MpduRepr, SeqNrRepr};

use dot15d4_frame::MpduWithIes;
use dot15d4_util::allocator::IntoBuffer;

use crate::pib::Pib;

/// Builder for Enhanced Beacon frames.
///
/// Pre-configures the MPDU representation with TSCH IEs for efficient
/// beacon generation. The IE structure is defined at compile time.
pub struct EnhancedBeaconBuilder<'buffer, RadioDriverImpl: DriverConfig> {
    /// Pre-configured MPDU representation with IE structure.
    mpdu_repr: MpduRepr<'buffer, MpduWithIes>,
    /// Phantom data for radio driver type.
    _phantom: PhantomData<RadioDriverImpl>,
}

impl<'buffer, RadioDriverImpl: DriverConfig> EnhancedBeaconBuilder<'buffer, RadioDriverImpl> {
    /// Create a new Enhanced Beacon builder.
    ///
    /// The builder is configured with the standard TSCH IEs:
    /// - TSCH Synchronization IE
    /// - Full TSCH Timeslot IE
    /// - TSCH Slotframe and Link IE
    pub const fn new() -> Self {
        static SLOTFRAMES: [u8; 1] = [1];
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

    /// Build an Enhanced Beacon frame with schedule information.
    ///
    /// Populates the beacon IEs with:
    /// - Source/destination addressing from PIB
    /// - Slotframe and link configuration from TSCH PIB
    /// - Timeslot timing parameters
    pub fn build_enhanced_beacon(
        &self,
        pib: &Pib,
        radio_frame: RadioFrame<RadioFrameUnsized>,
    ) -> Option<MpduFrame> {
        let buffer = radio_frame.into_buffer();

        // Create the frame writer
        let mut mpdu_writer = self
            .mpdu_repr
            .into_writer::<RadioDriverImpl>(FrameVersion::Ieee802154, FrameType::Beacon, 0, buffer)
            .ok()?;

        // Set addressing fields
        {
            let mut addressing = mpdu_writer.addressing_fields_mut();
            addressing.src_address_mut().set(&pib.extended_address);
            addressing
                .dst_address_mut()
                .set(&Address::<&[u8]>::BROADCAST_ADDR);
            addressing.dst_pan_id_mut().set(&pib.pan_id);
        }

        // Set IE content from schedule
        {
            let mut ies = mpdu_writer.ies_fields_mut();

            // Set Slotframe and Link IE from schedule
            if let Some(mut slotframe_ie) = ies.tsch_slotframe_link_mut() {
                let mut slotframes = slotframe_ie.slotframes_mut();

                // Iterate through schedule's slotframes
                for (sf_handle, sf_size) in pib.tsch.slotframe_info() {
                    if let Some(mut sf) = slotframes.next() {
                        sf.set_handle(sf_handle as u8);
                        sf.set_size(sf_size);

                        // Add links for this slotframe
                        let mut links = sf.links_mut();
                        for link in pib
                            .tsch
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
                let timings = &pib.tsch.timeslot_timings;
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

        Some(mpdu_writer.into_mpdu_frame())
    }
}

impl<'buffer, R: DriverConfig> Default for EnhancedBeaconBuilder<'buffer, R> {
    fn default() -> Self {
        Self::new()
    }
}
