#!/bin/sh -e
#
# Per-feature-combination compile matrix for the `dot15d4` crate.
#
# Why this exists
# ---------------
# The regular `cargo build` / `cargo test` jobs build the *whole workspace*, so
# Cargo feature unification turns on the union of every member's features. In
# particular `tsch` is pulled in transitively (via the examples / driver), which
# means a build that is broken *only* without `tsch` (or only with
# `tsch-coordinator`, etc.) compiles green in the workspace build and ships
# broken. That is exactly how the `expected_rx_framestart`/`validate_ack_frame`
# non-`tsch` breakage reached `develop`.
#
# This script compiles `dot15d4` *in isolation* (`-p dot15d4
# --no-default-features`) for each feature combination that has its own
# conditional code paths, so a combination that does not compile fails CI on its
# own line. Use `cargo check` (fast, no codegen) for the compile gate; the host
# test combinations are run separately by `test.sh`.
#
# Run from the workspace root:  ./feature-matrix.sh

CARGO="${CARGO:-cargo}"

# Compile-check `dot15d4` with the given feature set in isolation.
check() {
    desc="$1"
    features="$2"
    echo "CHECK(dot15d4): ${desc}  [--no-default-features --features=${features}]"
    if [ -z "${features}" ]; then
        "${CARGO}" check -p dot15d4 --no-default-features
    else
        "${CARGO}" check -p dot15d4 --no-default-features --features="${features}"
    fi
}

# --- Frame-feature axis (security / ies independently and together) ----------
check "no features"            ""
check "security only"          "security"
check "ies only"               "ies"
check "security + ies"         "security,ies"

# --- TSCH axis (the combinations that gate real code) ------------------------
# `tsch` WITHOUT `tsch-coordinator` is the combination most prone to bit-rot,
# since most development happens with the coordinator enabled.
check "tsch (no coordinator)"          "ies,tsch"
check "tsch-coordinator"               "ies,tsch,tsch-coordinator"
check "security + tsch"                "security,ies,tsch"
check "security + tsch-coordinator"    "security,ies,tsch,tsch-coordinator"

# --- TSCH sync opt-outs (each disables a distinct sync code path) ------------
check "tsch, no ack-sync"          "ies,tsch,no-tsch-ack-sync"
check "tsch, no frame-sync"        "ies,tsch,no-tsch-frame-sync"
check "tsch, no drift-adjustment"  "ies,tsch,no-tsch-drift-adjustment"
check "tsch, no sync at all"       "ies,tsch,no-tsch-ack-sync,no-tsch-frame-sync,no-tsch-drift-adjustment"

# --- std host build ----------------------------------------------------------
check "std"                        "std"
check "std + tsch-coordinator"     "std,security,ies,tsch,tsch-coordinator"

echo "feature-matrix: all combinations compiled"
