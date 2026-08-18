#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    dot15d4::fuzz_api::fuzz_tsch_step(data);
});
