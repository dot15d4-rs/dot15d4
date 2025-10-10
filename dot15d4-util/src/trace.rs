use core::ptr::null_mut;

use cty::{c_char, c_uint, c_ulong};
use systemview_target::SystemView;

rtos_trace::global_trace! {SystemView}

pub mod export {
    pub use systemview_target;
}

#[repr(u32)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum TraceOffset {
    Dot15d4 = 100,
    Dot15d4DriverExecutor = 200,
    Dot15d4DriverRadio = 300,
    Dot15d4Embassy = 400,
}

impl TraceOffset {
    pub const fn wrap(&self, offset: u32) -> u32 {
        *self as u32 + offset
    }
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct SystemviewModule {
    module_desc: *const c_char,
    num_events: c_ulong,
    event_offset: c_ulong,
    send_module_desc: Option<unsafe extern "C" fn()>,
    next_module: *mut SystemviewModule,
}

impl SystemviewModule {
    pub const fn new(module_desc: &'static str, num_events: c_ulong) -> Self {
        Self {
            module_desc: module_desc.as_ptr(),
            num_events,
            event_offset: 0,
            send_module_desc: None,
            next_module: null_mut(),
        }
    }

    #[inline(always)]
    pub fn event_offset(&self) -> c_ulong {
        self.event_offset
    }
}

unsafe extern "C" {
    fn SEGGER_SYSVIEW_RegisterModule(module: *mut SystemviewModule);

    fn SEGGER_SYSVIEW_RecordU32(event_id: c_uint, para: c_ulong);

    fn SEGGER_SYSVIEW_RecordU32x2(event_id: c_uint, para0: c_ulong, para1: c_ulong);

    fn SEGGER_SYSVIEW_RecordU32x3(event_id: c_uint, para0: c_ulong, para1: c_ulong, para2: c_ulong);
}

/// See SEGGER_SYSVIEW_RegisterModule.
///
/// # Safety
///
/// The argument must point to a module with static lifetime that's never moved.
#[inline(always)]
pub unsafe fn systemview_register_module(module: *mut SystemviewModule) {
    SEGGER_SYSVIEW_RegisterModule(module);
}

/// See SEGGER_SYSVIEW_RecordU32.
#[inline(always)]
pub fn systemview_record_u32(event_id: c_uint, para: c_ulong) {
    unsafe {
        SEGGER_SYSVIEW_RecordU32(event_id, para);
    }
}

/// See SEGGER_SYSVIEW_RecordU32x2.
#[inline(always)]
pub fn systemview_record_u32x2(event_id: c_uint, para0: c_ulong, para1: c_ulong) {
    unsafe {
        SEGGER_SYSVIEW_RecordU32x2(event_id, para0, para1);
    }
}

/// See SEGGER_SYSVIEW_RecordU32x3.
#[inline(always)]
pub fn systemview_record_u32x3(event_id: c_uint, para0: c_ulong, para1: c_ulong, para2: c_ulong) {
    unsafe {
        SEGGER_SYSVIEW_RecordU32x3(event_id, para0, para1, para2);
    }
}

#[macro_export]
macro_rules! instrument {
    (bare_metal cpu_freq: $sysclock_freq:literal Hz) => {{
        $crate::instrument!(_inner: $sysclock_freq);

        impl rtos_trace::RtosTraceOSCallbacks for Application {
            fn task_list() {}
            fn time() -> u64 {
                0
            }
        }

        rtos_trace::global_os_callbacks! {Application};

        start_tracing
    }};

    (embassy cpu_freq: $sysclock_freq:literal Hz) => {{
        $crate::instrument!(_inner: $sysclock_freq);
        start_tracing
    }};

    (_inner: $sysclock_freq:literal) => {
        use $crate::trace::export::systemview_target;

        struct Application;

        rtos_trace::global_application_callbacks! {Application}

        impl rtos_trace::RtosTraceApplicationCallbacks for Application {
            fn system_description() {
                systemview_target::send_system_desc_app_name!("dot15d4");
                systemview_target::send_system_desc_interrupt!(17, "RADIO");
                systemview_target::send_system_desc_interrupt!(22, "GPIOTE");
                systemview_target::send_system_desc_interrupt!(24, "TIMER0");
                systemview_target::send_system_desc_interrupt!(27, "RTC0");
                systemview_target::send_system_desc_interrupt!(36, "SWI0");
            }

            fn sysclock() -> u32 {
                $sysclock_freq
            }
        }

        static SYSTEMVIEW: systemview_target::SystemView = systemview_target::SystemView::new();

        fn start_tracing() {
            SYSTEMVIEW.init();

            #[cfg(feature = "log")] {
                log::set_logger(&SYSTEMVIEW).ok();
                log::set_max_level(log::LevelFilter::Info);
            }

            rtos_trace::trace::start();
        }
    };
}

pub use instrument;
