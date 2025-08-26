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
    }};

    (embassy cpu_freq: $sysclock_freq:literal Hz) => {{
        $crate::instrument!(_inner: $sysclock_freq);
    }};

    (_inner: $sysclock_freq:literal) => {
        use $crate::trace::export::systemview_target;

        struct Application;

        rtos_trace::global_application_callbacks! {Application}

        impl rtos_trace::RtosTraceApplicationCallbacks for Application {
            fn system_description() {
                systemview_target::send_system_desc_app_name!("dot15d4");
                systemview_target::send_system_desc_interrupt!(17, "RADIO");
                systemview_target::send_system_desc_interrupt!(24, "TIMER0");
                systemview_target::send_system_desc_interrupt!(27, "RTC0");
                systemview_target::send_system_desc_interrupt!(36, "SWI0");
            }

            fn sysclock() -> u32 {
                $sysclock_freq
            }
        }

        static SYSTEMVIEW: systemview_target::SystemView = systemview_target::SystemView::new();
        SYSTEMVIEW.init();

        log::set_logger(&SYSTEMVIEW).ok();
        log::set_max_level(log::LevelFilter::Info);

        rtos_trace::trace::start();
    };
}

pub use instrument;
