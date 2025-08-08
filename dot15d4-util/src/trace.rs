use systemview_target::SystemView;

rtos_trace::global_trace! {SystemView}

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

struct Application;
rtos_trace::global_application_callbacks! {Application}
impl rtos_trace::RtosTraceApplicationCallbacks for Application {
    fn system_description() {
        systemview_target::send_system_desc_app_name!("dot15d4");
        systemview_target::send_system_desc_interrupt!(17, "RADIO");
        systemview_target::send_system_desc_interrupt!(27, "RTC0");
    }

    fn sysclock() -> u32 {
        // TODO: This frequency is hardware-dependent.
        64_000_000
    }
}

pub fn instrument() {
    static SYSTEMVIEW: SystemView = SystemView::new();
    SYSTEMVIEW.init();
    log::set_logger(&SYSTEMVIEW).ok();
    log::set_max_level(log::LevelFilter::Info);
}
