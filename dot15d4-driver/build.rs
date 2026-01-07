use std::env;
use std::fs::File;
use std::io::Write as IoWrite;
use std::path::{Path, PathBuf};

fn main() {
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    examples(&out_dir);
}

fn examples(out_dir: &Path) {
    File::create(out_dir.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("examples/nrf/memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rerun-if-changed=examples/nrf/memory.x");
    println!("cargo:rustc-link-arg-examples=--nmagic");
    println!("cargo:rustc-link-arg-examples=-Tlink.x");
    #[cfg(feature = "defmt")]
    println!("cargo:rustc-link-arg-examples=-Tdefmt.x");
}
