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
        ("MAC_MIN_BE", ("u8", "3")),
        ("MAC_MAX_BE", ("u8", "5")),
        ("MAC_MAX_CSMA_BACKOFFS", ("u8", "4")),
        ("MAC_MAX_FRAME_RETRIES", ("u8", "3")),
        ("MAC_PAN_ID", ("u16", "0xbeef")),
        ("MAC_IMPLICIT_BROADCAST", ("bool", "false")),
        ("MAC_TSCH_MIN_BE", ("u8", "1")),
        ("MAC_TSCH_MAX_BE", ("u8", "7")),
        ("MAC_JOIN_METRIC", ("u16", "1")),
        ("MAC_NEIGHBOR_TABLE_SIZE", ("usize", "8")),
        ("MAC_ETX_NOACK_PENALTY", ("u16", "12")),
        ("MAC_DISCONNECT_TIME", ("u16", "0x00ff")),
        ("MAC_TSCH_MAX_LINKS", ("usize", "5")),
        ("MAC_TSCH_MAX_SLOTFRAMES", ("usize", "1")),
        ("MAC_TSCH_MAX_PENDING_OPERATIONS", ("usize", "5")),
        ("MAC_HOPPING_SEQUENCE_MAX_LENGTH", ("usize", "16")),
        ("MAC_TSCH_DRIFT_BUFFER_LEN", ("usize", "8")),
        ("MAC_TSCH_MAX_DRIFT_PPM", ("u32", "200")),
        ("MAC_TSCH_DRIFT_MIN_INTERVAL_SLOTS", ("u32", "100")),
        ("MAC_TSCH_GUARD_FLOOR_US", ("u16", "100")),
        ("MAC_TSCH_GUARD_SAFE_US", ("u16", "2200")),
        ("MAC_TSCH_GUARD_COMMIT_SYNCS", ("u8", "8")),
        ("MAC_TSCH_GUARD_COMMIT_MARGIN", ("u8", "1")),
        ("MAC_TSCH_KEEP_ALIVE_PERIOD", ("u16", "1500")),
        ("MAC_TSCH_MAX_KEEP_ALIVE_MISSES", ("u8", "4")),
        ("MAC_TSCH_GUARD_RELAX_MISSES", ("u8", "1")),
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

    data.push_str(concat!(
        "const _: () = assert!(MAC_MIN_BE <= MAC_MAX_BE, \"MAC_MIN_BE must be <= MAC_MAX_BE\");\n",
        "const _: () = assert!(MAC_MAX_BE >= 3 && MAC_MAX_BE <= 8, \"MAC_MAX_BE must be in 3..=8\");\n",
        "const _: () = assert!(MAC_MAX_CSMA_BACKOFFS <= 5, \"MAC_MAX_CSMA_BACKOFFS must be in 0..=5\");\n",
        "const _: () = assert!(MAC_MAX_FRAME_RETRIES <= 7, \"MAC_MAX_FRAME_RETRIES must be in 0..=7\");\n",
        "const _: () = assert!(MAC_TSCH_MIN_BE <= MAC_TSCH_MAX_BE, \"MAC_TSCH_MIN_BE must be <= MAC_TSCH_MAX_BE\");\n",
        "const _: () = assert!(MAC_TSCH_MAX_BE >= 3 && MAC_TSCH_MAX_BE <= 8, \"MAC_TSCH_MAX_BE must be in 3..=8\");\n",
        "const _: () = assert!(MAC_TSCH_MAX_KEEP_ALIVE_MISSES == 0 || MAC_TSCH_GUARD_RELAX_MISSES <= MAC_TSCH_MAX_KEEP_ALIVE_MISSES, \"MAC_TSCH_GUARD_RELAX_MISSES must be <= MAC_TSCH_MAX_KEEP_ALIVE_MISSES\");\n",
    ));

    // Now that we have the code of the configuration, actually write it to a file
    let out_file = out_dir.join("config.rs");
    std::fs::write(out_file, data).unwrap();
}
