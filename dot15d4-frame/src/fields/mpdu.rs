use dot15d4_driver::{
    frame::{
        AddressingFields, AddressingMode, AddressingRepr, FrameControl, FrameType, FrameVersion,
        RadioFrame, RadioFrameRepr, RadioFrameSized,
    },
    DriverConfig,
};
use dot15d4_util::{
    allocator::{BufferToken, IntoBuffer},
    frame::{Frame, FramePdu},
    Error, Result as SimplifiedResult,
};

#[cfg(feature = "ies")]
use crate::repr::IeListRepr;
use crate::{
    mpdu::MpduFrame,
    repr::{MpduRepr, SeqNrRepr},
    MpduParsedUpToAddressing, MpduParsedUpToSecurity, MpduWithAddressing, MpduWithAllFields,
    MpduWithFrameControl, MpduWithIes, MpduWithSecurity,
};

use super::field_ranges::MpduFieldRanges;

/// Accessors into fields that are available on an unparsed MPDU frame.
impl MpduFrame {
    fn frame_control_slice_ref(&self) -> &[u8] {
        let mpdu_field_ranges = MpduFieldRanges::new(self.offset, SeqNrRepr::No);
        &self.buffer[mpdu_field_ranges.range_frame_control()]
    }

    pub(crate) fn frame_control_slice_mut(&mut self) -> &mut [u8] {
        let mpdu_field_ranges = MpduFieldRanges::new(self.offset, SeqNrRepr::No);
        &mut self.buffer[mpdu_field_ranges.range_frame_control()]
    }

    /// Provides read-only access to the [`FrameControl`] field.
    pub fn frame_control(&self) -> FrameControl<&[u8]> {
        FrameControl::new_unchecked(self.frame_control_slice_ref())
    }

    /// Provides mutable access to the [`FrameControl`] field.
    pub fn frame_control_mut(&mut self) -> FrameControl<&mut [u8]> {
        FrameControl::new_unchecked(self.frame_control_slice_mut())
    }

    /// Reads the sequence number field.
    pub fn sequence_number(&self) -> Option<u8> {
        if self.frame_control().sequence_number_suppression() {
            return None;
        }

        // Safety: We now know for sure that a sequence number is present.
        let mpdu_field_ranges = MpduFieldRanges::new(self.offset, SeqNrRepr::Yes);
        Some(self.buffer[mpdu_field_ranges.offset_seq_nr().get() as usize])
    }

    /// Writes the sequence number field.
    ///
    /// Safety: Requires the sequence number suppression field of the frame
    ///         control field to be false.
    pub fn set_sequence_number(&mut self, seq_nr: u8) -> SimplifiedResult<()> {
        if self.frame_control().sequence_number_suppression() {
            return Err(Error);
        }

        let mpdu_field_ranges = MpduFieldRanges::new(self.offset, SeqNrRepr::Yes);
        self.buffer[mpdu_field_ranges.offset_seq_nr().get() as usize] = seq_nr;

        Ok(())
    }

    /// Initializes a partially parsed MPDU with read-only access to the frame
    /// control and sequence number fields from an unparsed MPDU.
    ///
    /// This is used to start parsing an incoming MPDU.
    pub fn reader(&self) -> MpduParser<&MpduFrame, MpduWithFrameControl> {
        MpduParser {
            mpdu_field_ranges: MpduFieldRanges::new(
                self.offset,
                if self.frame_control().sequence_number_suppression() {
                    SeqNrRepr::No
                } else {
                    SeqNrRepr::Yes
                },
            ),
            mpdu: self,
        }
    }

    /// Initializes a partially parsed MPDU with read-only access to the frame
    /// control and sequence number fields from an unparsed MPDU.
    ///
    /// This is used to start parsing an incoming MPDU.
    pub fn writer(&mut self) -> MpduParser<&mut MpduFrame, MpduWithFrameControl> {
        MpduParser {
            mpdu_field_ranges: MpduFieldRanges::new(
                self.offset,
                if self.frame_control().sequence_number_suppression() {
                    SeqNrRepr::No
                } else {
                    SeqNrRepr::Yes
                },
            ),
            mpdu: self,
        }
    }

    /// Consumes the MPDU frame and produces an owning MPDU parser instead.
    pub fn into_parser(self) -> MpduParser<MpduFrame, MpduWithFrameControl> {
        MpduParser {
            mpdu_field_ranges: MpduFieldRanges::new(
                self.offset,
                if self.frame_control().sequence_number_suppression() {
                    SeqNrRepr::No
                } else {
                    SeqNrRepr::Yes
                },
            ),
            mpdu: self,
        }
    }
}

/// An MPDU parser that provides staged access to frame content depending on its
/// parsing state.
///
/// The level of access (owned, read-only, read/write) is defined by the type of
/// the `AnyMpdu` generic.
pub struct MpduParser<AnyMpdu, State> {
    mpdu_field_ranges: MpduFieldRanges<State>,
    mpdu: AnyMpdu,
}

impl<'repr> MpduRepr<'repr, MpduWithIes> {
    /// Allocates and initializes an MPDU reader/writer based on this MPDU
    /// representation.
    ///
    /// This is used to build an MPDU from scratch.
    ///
    /// All structural information that can be derived from the MPDU
    /// representation or function arguments will be written into the buffer:
    /// - All sub-fields of the frame control field will be initialized with
    ///   known values except "Frame Pending" and "AR" which will be initialized
    ///   to zero.
    /// - The security control field of the auxiliary security header will be
    ///   initialized.
    /// - All header, payload and nested IEs will be pre-initialized with their
    ///   length, id and type. Required termination IEs will be identified and
    ///   added automatically.
    /// - Information required to determine the structure and size of
    ///   dynamically sized IEs will be pre-initialized, namely the number of
    ///   channels and the extended bitmap length for the long version of a
    ///   Channel Hopping IE and the number of slotframes and links within each
    ///   slotframe for a TSCH Slotframe and Link IE.
    pub fn into_parsed_mpdu<Config: DriverConfig>(
        &self,
        frame_version: FrameVersion,
        frame_type: FrameType,
        frame_payload_length: u16,
        // Note: The buffer may or may not be zeroed.
        buffer: BufferToken,
    ) -> Result<MpduParser<MpduFrame, MpduWithAllFields>, BufferToken> {
        let mpdu_length_wo_fcs = match self.mpdu_length_wo_fcs(frame_payload_length) {
            Ok(mpdu_length_wo_fcs) => mpdu_length_wo_fcs,
            Err(_) => return Err(buffer),
        };

        let radio_frame_repr = RadioFrameRepr::<Config, RadioFrameSized>::new(mpdu_length_wo_fcs);
        let min_buffer_length = radio_frame_repr.pdu_length();
        assert!(buffer.len() >= min_buffer_length as usize);

        let offset_mpdu = radio_frame_repr.headroom_length();
        let mut mpdu = MpduFrame::new(buffer, offset_mpdu, mpdu_length_wo_fcs);

        let (dst_addr_mode, src_addr_mode, pan_id_compression) = match &self.addressing {
            Some(addressing) => (
                addressing.dst_addr_mode(),
                addressing.src_addr_mode(),
                addressing.pan_id_compression(),
            ),
            None => (AddressingMode::Absent, AddressingMode::Absent, false),
        };

        #[cfg(feature = "security")]
        let security_enabled = self.security.is_some();
        #[cfg(not(feature = "security"))]
        let security_enabled = false;

        #[cfg(feature = "ies")]
        let ie_present = !self.ies.is_empty();
        #[cfg(not(feature = "ies"))]
        let ie_present = false;

        // All frame control fields shall default to zero.
        mpdu.frame_control_slice_mut().fill(0);

        let mut fc = mpdu.frame_control_mut();
        fc.set_frame_version(frame_version);
        fc.set_frame_type(frame_type);
        fc.set_sequence_number_suppression(matches!(self.seq_nr, SeqNrRepr::No));
        fc.set_pan_id_compression(pan_id_compression);
        fc.set_dst_addressing_mode(dst_addr_mode);
        fc.set_src_addressing_mode(src_addr_mode);
        fc.set_security_enabled(security_enabled);
        fc.set_information_elements_present(ie_present);

        // TODO: Initialize other fields mentioned in the method docs.

        let mpdu_field_ranges = MpduFieldRanges::new(offset_mpdu, self.seq_nr);

        let mpdu_field_ranges = if let Some(addressing) = self.addressing {
            match mpdu_field_ranges.with_addressing(addressing) {
                Ok(mpdu_field_ranges) => mpdu_field_ranges,
                Err(_) => return Err(mpdu.into_buffer()),
            }
        } else {
            mpdu_field_ranges.without_addressing()
        };

        #[cfg(feature = "security")]
        let mpdu_field_ranges = if let Some(security) = self.security {
            mpdu_field_ranges.with_security(security)
        } else {
            mpdu_field_ranges.without_security()
        };
        #[cfg(not(feature = "security"))]
        let mpdu_field_ranges = mpdu_field_ranges.without_security();

        #[cfg(feature = "ies")]
        let mpdu_field_ranges = match self.ies {
            IeListRepr::Empty => {
                mpdu_field_ranges.without_ies_with_payload_length::<Config>(frame_payload_length)
            }
            ies => match mpdu_field_ranges
                .with_ies_and_payload_length::<Config>(ies, frame_payload_length)
            {
                Ok(ies) => ies,
                Err(_) => return Err(mpdu.into_buffer()),
            },
        };
        #[cfg(not(feature = "ies"))]
        let mpdu_field_ranges =
            mpdu_field_ranges.without_ies_with_payload_length::<Config>(frame_payload_length);

        Ok(MpduParser {
            mpdu_field_ranges,
            mpdu,
        })
    }
}

/// A read-only proxy to fields accessible from an MPDU in any parsing state.
impl<ReadOnlyMpdu: AsRef<MpduFrame>, State> MpduParser<ReadOnlyMpdu, State> {
    /// See [`MpduFrame::frame_control()`].
    ///
    /// Note: Only read access to the full frame control field is provided as
    ///       the frame may no longer be changed structurally at this stage.
    pub fn frame_control(&self) -> FrameControl<&[u8]> {
        self.mpdu.as_ref().frame_control()
    }

    /// See [`MpduFrame::sequence_number()`].
    pub fn sequence_number(&self) -> Option<u8> {
        self.mpdu.as_ref().sequence_number()
    }

    /// Check whether the frame is a valid IEEE 802.15.4 frame.
    ///
    /// TODO: This is part of the incoming frame procedure. Verify that all
    ///       checks are properly executed here.
    pub fn is_valid(&self) -> bool {
        self.frame_control().is_valid()
    }
}

/// A write-only proxy to fields accessible from an MPDU in any parsing state.
///
/// Note: Only parts of the frame control field that don't change the MPDU
///       structurally may be edited. All other fields must be initialized via
///       [`MpduRepr::into_parsed_mpdu()`].
impl<WriteOnlyMpdu: AsMut<MpduFrame>, State> MpduParser<WriteOnlyMpdu, State> {
    /// See [`FrameControl::set_ack_request()`]
    pub fn set_ack_request(&mut self, ack_request: bool) {
        self.mpdu
            .as_mut()
            .frame_control_mut()
            .set_ack_request(ack_request);
    }

    /// See [`FrameControl::set_frame_pending()`]
    pub fn set_frame_pending(&mut self, frame_pending: bool) {
        self.mpdu
            .as_mut()
            .frame_control_mut()
            .set_frame_pending(frame_pending);
    }

    /// See [`MpduFrame::set_sequence_number()`].
    pub fn set_sequence_number(&mut self, seq_nr: u8) -> SimplifiedResult<()> {
        // Safety: We synchronize the frame representation and the frame
        //         control field when we instantiate the parsed MPDU. The
        //         following call checks the frame control field's sequence
        //         number suppression field.
        self.mpdu.as_mut().set_sequence_number(seq_nr)
    }
}

/// Methods available for parsers that own the underlying MPDU.
impl<State> MpduParser<MpduFrame, State> {
    /// Once all reading/writing has been done, convert the frame back into
    /// an MPDU frame.
    ///
    /// Usually required when parsing incoming frames on the Rx path.
    ///
    /// Note: This will drop all parsing information. Only do this once you're
    ///       sure field access is no longer needed.
    pub fn into_mpdu_frame(self) -> MpduFrame {
        self.mpdu
    }

    /// Once all reading/writing has been done, convert the frame into a radio
    /// frame.
    ///
    /// Usually required when building frames from scratch for Tx.
    ///
    /// Note: This will drop all parsing information. Only do this once you're
    ///       sure the frame has been finalized.
    pub fn into_radio_frame<Config: DriverConfig>(self) -> RadioFrame<RadioFrameSized> {
        self.mpdu.into_radio_frame::<Config>()
    }
}

/// MPDU reader at the initial frame-control parsing stage.
impl<ReadOnlyMpdu: AsRef<MpduFrame>> MpduParser<ReadOnlyMpdu, MpduWithFrameControl> {
    /// Parses the frame control field to identify the addressing configuration
    /// of the MPDU.
    pub fn parse_addressing(
        self,
    ) -> SimplifiedResult<MpduParser<ReadOnlyMpdu, MpduWithAddressing>> {
        let addressing = match AddressingRepr::from_frame_control(self.frame_control()) {
            Ok(result) => result,
            Err(_) => return Err(Error),
        };
        let mpdu_field_ranges = if let Some(addressing) = addressing {
            match self.mpdu_field_ranges.with_addressing(addressing) {
                Ok(result) => result,
                Err(_) => return Err(Error),
            }
        } else {
            self.mpdu_field_ranges.without_addressing()
        };

        Ok(MpduParser {
            mpdu_field_ranges,
            mpdu: self.mpdu,
        })
    }
}

/// MPDU writer at the initial frame-control parsing stage.
impl<ReadWriteMpdu: AsRef<MpduFrame> + AsMut<MpduFrame>>
    MpduParser<ReadWriteMpdu, MpduWithFrameControl>
{
    /// Parses the frame control field to identify the addressing configuration
    /// of the MPDU.
    pub fn parse_addressing_mut(
        self,
    ) -> SimplifiedResult<MpduParser<ReadWriteMpdu, MpduWithAddressing>> {
        let addressing = match AddressingRepr::from_frame_control(self.frame_control()) {
            Ok(result) => result,
            Err(_) => return Err(Error),
        };
        let mpdu_field_ranges = if let Some(addressing) = addressing {
            match self.mpdu_field_ranges.with_addressing(addressing) {
                Ok(result) => result,
                Err(_) => return Err(Error),
            }
        } else {
            self.mpdu_field_ranges.without_addressing()
        };

        Ok(MpduParser {
            mpdu_field_ranges,
            mpdu: self.mpdu,
        })
    }
}

/// MPDU reader at the addressing parsing stage.
impl<ReadOnlyMpdu: AsRef<MpduFrame>> MpduParser<ReadOnlyMpdu, MpduWithAddressing> {
    /// Parses the frame control field to identify the security configuration of
    /// the MPDU.
    pub fn parse_security(self) -> MpduParser<ReadOnlyMpdu, MpduWithSecurity> {
        // TODO: implement
        debug_assert!(!self.frame_control().security_enabled());

        MpduParser {
            mpdu_field_ranges: self.mpdu_field_ranges.without_security(),
            mpdu: self.mpdu,
        }
    }
}

/// MPDU reader at the security parsing stage.
impl<ReadOnlyMpdu: AsRef<MpduFrame>> MpduParser<ReadOnlyMpdu, MpduWithSecurity> {
    /// Parses the frame control and information element fields to identify the
    /// information elements of the MPDU.
    pub fn parse_ies<Config: DriverConfig>(
        self,
    ) -> SimplifiedResult<MpduParser<ReadOnlyMpdu, MpduWithAllFields>> {
        // TODO: implement
        debug_assert!(!self.frame_control().information_elements_present());

        let mpdu_length_wo_fcs = self.mpdu.as_ref().pdu_length_wo_fcs();
        let mpdu_field_ranges = match self
            .mpdu_field_ranges
            .without_ies_with_mpdu_length::<Config>(mpdu_length_wo_fcs)
        {
            Ok(result) => result,
            Err(_) => return Err(Error),
        };

        Ok(MpduParser {
            mpdu_field_ranges,
            mpdu: self.mpdu,
        })
    }
}

/// Exposes read-only fields accessible from an MPDU at the addressing
/// parsing stage or beyond.
impl<ReadOnlyMpdu: AsRef<MpduFrame>, State: MpduParsedUpToAddressing>
    MpduParser<ReadOnlyMpdu, State>
{
    /// Read-only addressing field access.
    pub fn addressing_fields(&self) -> SimplifiedResult<Option<AddressingFields<&[u8]>>> {
        let range_addressing = self.mpdu_field_ranges.range_addressing().ok_or(Error)?;
        let addressing_repr =
            AddressingRepr::from_frame_control(self.frame_control())?.ok_or(Error)?;

        // Safety: Addressing representation and range are both synced with the
        //         frame control field.
        let addressing_fields = unsafe {
            AddressingFields::new_unchecked(
                &self.mpdu.as_ref().buffer[range_addressing],
                addressing_repr,
            )?
        };

        Ok(Some(addressing_fields))
    }
}

/// Exposes writable fields accessible from an MPDU at the addressing
/// parsing stage or beyond.
impl<ReadWriteMpdu: AsRef<MpduFrame> + AsMut<MpduFrame>, State: MpduParsedUpToAddressing>
    MpduParser<ReadWriteMpdu, State>
{
    /// Writable addressing field access.
    pub fn addressing_fields_mut(
        &mut self,
    ) -> SimplifiedResult<Option<AddressingFields<&mut [u8]>>> {
        let range_addressing = self.mpdu_field_ranges.range_addressing().ok_or(Error)?;
        let addressing_repr =
            AddressingRepr::from_frame_control(self.frame_control())?.ok_or(Error)?;

        // Safety: Addressing representation and range are both synced with the
        //         frame control field.
        let addressing_fields = unsafe {
            AddressingFields::new_unchecked(
                &mut self.mpdu.as_mut().buffer[range_addressing],
                addressing_repr,
            )?
        };

        Ok(Some(addressing_fields))
    }
}

/// Allows access to stand-alone addressing field references.
impl<'mpdu, State: MpduParsedUpToAddressing> MpduParser<&'mpdu MpduFrame, State> {
    /// Consumes an MPDU reader and returns a stand-alone reference into the
    /// MPDU's addressing fields instead.
    pub fn into_addressing_fields(self) -> SimplifiedResult<Option<AddressingFields<&'mpdu [u8]>>> {
        let range_addressing = self.mpdu_field_ranges.range_addressing().ok_or(Error)?;
        let addressing_repr =
            AddressingRepr::from_frame_control(self.frame_control())?.ok_or(Error)?;

        // Safety: Addressing representation and range are both synced with the
        //         frame control field.
        let addressing_fields = unsafe {
            AddressingFields::new_unchecked(&self.mpdu.buffer[range_addressing], addressing_repr)?
        };

        Ok(Some(addressing_fields))
    }
}

/// Exposes fields accessible from an MPDU at the security parsing stage or
/// beyond.
impl<AnyMpdu, State: MpduParsedUpToSecurity> MpduParser<AnyMpdu, State> {
    // TODO: Add access to the aux security header and MIC.
}

/// Exposes read-only fields accessible from an MPDU once it is fully parsed.
impl<ReadOnlyMpdu: AsRef<MpduFrame>> MpduParser<ReadOnlyMpdu, MpduWithAllFields> {
    // TODO: Add access to IEs.

    pub fn frame_payload(&self) -> Option<&[u8]> {
        Some(&self.mpdu.as_ref().buffer[self.mpdu_field_ranges.range_frame_payload()?])
    }

    pub fn fcs(&self) -> Option<&[u8]> {
        Some(&self.mpdu.as_ref().buffer[self.mpdu_field_ranges.range_fcs()?])
    }
}

/// Exposes write-only fields accessible from an MPDU once it is fully parsed.
impl<ReadOnlyMpdu: AsMut<MpduFrame>> MpduParser<ReadOnlyMpdu, MpduWithAllFields> {
    // TODO: Add access to IEs.

    pub fn frame_payload_mut(&mut self) -> Option<&mut [u8]> {
        Some(&mut self.mpdu.as_mut().buffer[self.mpdu_field_ranges.range_frame_payload()?])
    }

    pub fn fcs_mut(&mut self) -> Option<&mut [u8]> {
        Some(&mut self.mpdu.as_mut().buffer[self.mpdu_field_ranges.range_fcs()?])
    }
}

impl<State> IntoBuffer for MpduParser<MpduFrame, State> {
    fn into_buffer(self) -> BufferToken {
        self.mpdu.into_buffer()
    }
}

impl<State> FramePdu for MpduParser<MpduFrame, State> {
    type Pdu = Self;

    fn pdu_ref(&self) -> &Self {
        self
    }

    fn pdu_mut(&mut self) -> &mut Self {
        self
    }
}

impl Frame for MpduParser<MpduFrame, MpduWithAllFields> {
    /// Provides access to the frame payload for reading.
    fn sdu_ref(&self) -> &[u8] {
        self.frame_payload().unwrap_or(&[])
    }

    /// Provides access to the frame payload for writing.
    fn sdu_mut(&mut self) -> &mut [u8] {
        self.frame_payload_mut().unwrap_or(&mut [])
    }
}
