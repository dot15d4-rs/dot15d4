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

use crate::constants::MAC_HOPPING_SEQUENCE_MAX_LENGTH;
use crate::pib::Pib;

/// Builder for Enhanced Beacon frames.
pub struct EnhancedBeaconBuilder<'buffer, RadioDriverImpl: DriverConfig> {
    _phantom: PhantomData<(&'buffer (), RadioDriverImpl)>,
}

impl<'buffer, RadioDriverImpl: DriverConfig> EnhancedBeaconBuilder<'buffer, RadioDriverImpl> {
    /// Create a new Enhanced Beacon builder.
    pub const fn new() -> Self {
        Self {
            _phantom: PhantomData,
        }
    }

    pub fn build_enhanced_beacon(
        &self,
        pib: &Pib,
        radio_frame: RadioFrame<RadioFrameUnsized>,
    ) -> Option<MpduFrame> {
        let buffer = radio_frame.into_buffer();

        let hopping_len = pib.hopping_sequence.len().min(MAC_HOPPING_SEQUENCE_MAX_LENGTH);

        let slotframes: [u8; 1] = [1];
        let ies: [IeRepr; 4] = [
            IeRepr::TschSynchronizationNestedIe,
            IeRepr::FullTschTimeslotNestedIe,
            IeRepr::TschSlotframeAndLinkNestedIe(&slotframes),
            IeRepr::FullChannelHoppingNestedIe(hopping_len as u8, false),
        ];
        let ie_repr_list = IeReprList::new(&ies);
        let ie_list = IeListRepr::WithoutTerminationIes(ie_repr_list);

        let mpdu_repr: MpduRepr<'_, MpduWithIes> = MpduRepr::new()
            .with_frame_control(SeqNrRepr::No)
            .with_addressing(AddressingRepr::new(
                AddressingMode::Short,
                AddressingMode::Extended,
                false,
                PanIdCompressionRepr::Yes,
            ))
            .without_security()
            .with_ies(ie_list);

        // Create the frame writer
        let mut mpdu_writer = mpdu_repr
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

            if let Some(mut ch_hop_ie) = ies.channel_hopping_mut() {
                ch_hop_ie.set_hopping_sequence_id(1);
                ch_hop_ie.set_channel_page(0);
                ch_hop_ie.set_num_channels(16);
                ch_hop_ie.set_phy_configuration(0);
                ch_hop_ie.set_hopping_sequence_length(hopping_len as u16);
                let mut channels = [0u8; MAC_HOPPING_SEQUENCE_MAX_LENGTH];
                for (slot, channel) in channels.iter_mut().zip(pib.hopping_sequence.iter()) {
                    *slot = u8::from(*channel);
                }
                ch_hop_ie.set_hopping_sequence_list(&channels[..hopping_len]);
                ch_hop_ie.set_current_hop(0);
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
