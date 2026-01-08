use std::collections::HashMap;
use std::env;
use std::fmt::Write as FmtWrite;
use std::path::{Path, PathBuf};

fn main() {
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    const_config(&out_dir);
}

fn const_config(out_dir: &Path) {
    // (Variable, Type, Default value)
    // TODO: Set the default PAN ID to 0xffff once we implement association.
    let mut const_config: HashMap<&str, (&str, &str)> = HashMap::from([
        ("MAC_MIN_BE", ("u8", "0")),
        ("MAC_MAX_BE", ("u8", "8")),
        ("MAC_MAX_CSMA_BACKOFFS", ("u8", "16")),
        ("MAC_MAX_FRAME_RETRIES", ("u8", "3")),
        ("MAC_PAN_ID", ("u16", "0xbeef")),
        ("MAC_IMPLICIT_BROADCAST", ("bool", "false")),
        ("MAC_TSCH_MIN_BE", ("u8", "1")),
        ("MAC_TSCH_MAX_BE", ("u8", "7")),
        ("MAC_JOIN_METRIC", ("u16", "1")),
        ("MAC_DISCONNECT_TIME", ("u16", "0x00ff")),
        ("MAC_TSCH_MAX_LINKS", ("usize", "5")),
        ("MAC_TSCH_MAX_SLOTFRAMES", ("usize", "1")),
        ("MAC_TSCH_MAX_PENDING_OPERATIONS", ("usize", "5")),
        ("MAC_HOPPING_SEQUENCE_MAX_LENGTH", ("usize", "16")),
    ]);

    // Make sure we get rerun if needed
    println!("cargo:rerun-if-changed=build.rs");
    for name in const_config.keys() {
        println!("cargo:rerun-if-env-changed=DOT15D4_{name}");
    }

    // Collect environment variables
    let mut data = String::new();
    // Write preamble

    for (var, value) in std::env::vars() {
        if let Some(name) = var.strip_prefix("DOT15D4_") {
            // discard from hashmap as a way of consuming the setting
            let Some((_, (ty, _))) = const_config.remove_entry(name) else {
                panic!("Wrong configuration name {name}");
            };

            // write to file
            writeln!(data, "pub const {name}: {ty} = {value};").unwrap();
        }
    }

    // Take the remaining configs and write the default value to the file
    for (name, (ty, value)) in const_config.iter() {
        writeln!(data, "pub const {name}: {ty} = {value};").unwrap();
    }

    // Now that we have the code of the configuration, actually write it to a file
    let out_file = out_dir.join("config.rs");
    std::fs::write(out_file, data).unwrap();
}
