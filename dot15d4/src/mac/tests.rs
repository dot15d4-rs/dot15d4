//! Test infrastructure and tests for MAC task state machines.

#![allow(dead_code)]

use dot15d4_driver::{
    radio::frame::{RadioFrame, RadioFrameSized, RadioFrameUnsized},
    timer::NsInstant,
};
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_util::allocator::{BufferToken, IntoBuffer};

use crate::{
    mac::MacBufferAllocator,
    scheduler::{
        tests::{create_test_allocator, FakeDriverConfig},
        MacCommandType, ReceptionType, SchedulerCommandResult, SchedulerReceptionResult,
        SchedulerRequest, SchedulerResponse, SchedulerTransmissionResult,
    },
};

use super::task::{MacTask, MacTaskEvent, MacTaskTransition};

// ============================================================================
// Step Outcome
// ============================================================================

/// Outcome of a single MAC task step.
pub(crate) enum MacStepOutcome<TaskResult> {
    /// Task produced a scheduler request and continues.
    Pending {
        request: SchedulerRequest,
        intermediate_result: Option<TaskResult>,
    },
    /// Task terminated with a final result.
    Terminated(TaskResult),
}

impl<R> MacStepOutcome<R> {
    /// Unwrap a pending request, panicking if terminated.
    pub fn unwrap_request(self) -> SchedulerRequest {
        match self {
            MacStepOutcome::Pending { request, .. } => request,
            MacStepOutcome::Terminated(_) => panic!("expected Pending, got Terminated"),
        }
    }

    /// Unwrap a terminated result, panicking if pending.
    pub fn unwrap_terminated(self) -> R {
        match self {
            MacStepOutcome::Terminated(r) => r,
            MacStepOutcome::Pending { .. } => panic!("expected Terminated, got Pending"),
        }
    }

    pub fn is_terminated(&self) -> bool {
        matches!(self, MacStepOutcome::Terminated(_))
    }

    pub fn is_pending(&self) -> bool {
        matches!(self, MacStepOutcome::Pending { .. })
    }
}

// ============================================================================
// Request Classification
// ============================================================================

/// Classification of a SchedulerRequest for test assertions.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum RequestType {
    Transmission,
    Reception(ReceptionType),
    Command,
}

impl From<&SchedulerRequest> for RequestType {
    fn from(req: &SchedulerRequest) -> Self {
        match req {
            SchedulerRequest::Transmission(_) => RequestType::Transmission,
            SchedulerRequest::Reception(r) => RequestType::Reception(*r),
            SchedulerRequest::Command(_) => RequestType::Command,
        }
    }
}

// ============================================================================
// MAC Task Test Runner
// ============================================================================

/// Synchronous test runner for MAC task state machines.
pub struct MacTaskTestRunner<Task: MacTask> {
    task: Option<Task>,
    allocator: MacBufferAllocator,
    /// Buffers that need deallocation on drop.
    pending_buffers: Vec<BufferToken>,
}

impl<Task: MacTask> MacTaskTestRunner<Task> {
    pub fn new(task: Task, allocator: MacBufferAllocator) -> Self {
        Self {
            task: Some(task),
            allocator,
            pending_buffers: Vec::new(),
        }
    }

    /// Whether the task has terminated.
    pub fn is_terminated(&self) -> bool {
        self.task.is_none()
    }

    /// Reference to the buffer allocator.
    pub fn allocator(&self) -> &MacBufferAllocator {
        &self.allocator
    }

    // ========================================================================
    // Step Methods
    // ========================================================================

    /// Step the task with a given event.
    pub fn step(&mut self, event: MacTaskEvent) -> MacStepOutcome<Task::Result> {
        let task = self.task.take().expect("task already terminated");
        match task.step(event) {
            MacTaskTransition::SchedulerRequest(next_task, request, intermediate) => {
                self.task = Some(next_task);
                MacStepOutcome::Pending {
                    request,
                    intermediate_result: intermediate,
                }
            }
            MacTaskTransition::Terminated(result) => MacStepOutcome::Terminated(result),
        }
    }

    /// Step the task with Entry event.
    pub fn step_entry(&mut self) -> MacStepOutcome<Task::Result> {
        self.step(MacTaskEvent::Entry)
    }

    /// Step the task with a scheduler response.
    pub fn step_response(&mut self, response: SchedulerResponse) -> MacStepOutcome<Task::Result> {
        self.step(MacTaskEvent::SchedulerResponse(response))
    }

    // ========================================================================
    // Buffer Management
    // ========================================================================

    /// Track a buffer for cleanup on drop.
    pub fn track_buffer(&mut self, buffer: BufferToken) {
        self.pending_buffers.push(buffer);
    }

    /// Track an MpduFrame's buffer for cleanup.
    pub fn track_mpdu(&mut self, mpdu: MpduFrame) {
        self.pending_buffers.push(mpdu.into_buffer());
    }

    /// Track buffers owned by RadioFrames.
    pub fn track_radio_frame_sized(&mut self, frame: RadioFrame<RadioFrameSized>) {
        self.pending_buffers.push(frame.into_buffer());
    }

    pub fn track_radio_frame_unsized(&mut self, frame: RadioFrame<RadioFrameUnsized>) {
        self.pending_buffers.push(frame.into_buffer());
    }

    // ========================================================================
    // Response Builders
    // ========================================================================

    /// Create a sized radio frame for use in responses.
    pub fn create_sized_radio_frame(&self, size: u16) -> RadioFrame<RadioFrameSized> {
        crate::scheduler::tests::create_sized_radio_frame(&self.allocator, size)
    }

    /// Create an unsized radio frame for use in responses.
    pub fn create_unsized_radio_frame(&self) -> RadioFrame<RadioFrameUnsized> {
        crate::scheduler::tests::create_unsized_radio_frame(&self.allocator)
    }
}

impl<Task: MacTask> Drop for MacTaskTestRunner<Task> {
    fn drop(&mut self) {
        for buffer in self.pending_buffers.drain(..) {
            unsafe {
                self.allocator.deallocate_buffer(buffer);
            }
        }
    }
}

// ============================================================================
// Response Helper Functions
// ============================================================================

/// Build a SchedulerResponse::Transmission(Sent) from an MpduFrame.
pub(crate) fn sent_response(mpdu: MpduFrame, instant: NsInstant) -> SchedulerResponse {
    let radio_frame = mpdu
        .into_radio_frame::<FakeDriverConfig>()
        .forget_size::<FakeDriverConfig>();
    SchedulerResponse::Transmission(SchedulerTransmissionResult::Sent(radio_frame, instant))
}

/// Build a SchedulerResponse::Transmission(NoAck) from an MpduFrame.
pub(crate) fn noack_response(mpdu: MpduFrame, instant: NsInstant) -> SchedulerResponse {
    let radio_frame = mpdu.into_radio_frame::<FakeDriverConfig>();
    SchedulerResponse::Transmission(SchedulerTransmissionResult::NoAck(radio_frame, instant))
}

/// Build a SchedulerResponse::Transmission(ChannelAccessFailure) from an MpduFrame.
pub(crate) fn channel_access_failure_response(mpdu: MpduFrame) -> SchedulerResponse {
    let radio_frame = mpdu.into_radio_frame::<FakeDriverConfig>();
    SchedulerResponse::Transmission(SchedulerTransmissionResult::ChannelAccessFailure(
        radio_frame,
    ))
}

/// Build a beacon reception response from a sized radio frame.
pub(crate) fn beacon_reception_response(
    frame: RadioFrame<RadioFrameSized>,
    instant: NsInstant,
) -> SchedulerResponse {
    SchedulerResponse::Reception(SchedulerReceptionResult::Beacon(frame, instant))
}

/// Build a command reception response from a sized radio frame.
pub(crate) fn mac_command_reception_response(
    frame: RadioFrame<RadioFrameSized>,
    instant: NsInstant,
) -> SchedulerResponse {
    SchedulerResponse::Reception(SchedulerReceptionResult::Command(frame, instant))
}

/// Extract the MpduFrame from a Transmission request.
pub(crate) fn extract_transmission_mpdu(request: SchedulerRequest) -> MpduFrame {
    match request {
        SchedulerRequest::Transmission(mpdu) => mpdu,
        _ => panic!("expected Transmission request"),
    }
}
