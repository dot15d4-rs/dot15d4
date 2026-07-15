# TSCH Example

`tsch-node` is a single binary that can play every role in a small TSCH
network. Role, topology and traffic are fixed at compile time through cargo
features and environment variables (read with `option_env!`, so set them on
the `cargo` command line). The cell layouts and the network formation
procedure live in [`schedule.rs`](schedule.rs).

All commands assume `cd examples/nrf52840`.

## Configuration

Cargo features:

| Feature | Meaning |
|---|---|
| `tsch` | Required by this example. |
| `tsch-coordinator` | Accept association requests from child nodes. |
| `tsch-association` | Perform the MLME association handshake with the parent after joining. The parent must be built with `tsch-coordinator`. |
| `defmt` (or `log`) | RTT log output. Needed to read the node's address at boot. |
| `tsch-log` | Trace the TSCH state machine (slot scheduling, link selection, backoff). |
| `sync-error-log` | Log the per-resync synchronization error. |
| `no-tsch-ack-sync`, `no-tsch-frame-sync`, `no-tsch-drift-adjustment` | Disable individual synchronization mechanisms. |

Environment variables (compile time):

| Variable | Default | Meaning |
|---|---|---|
| `NODE_ID` | `0` | Position in the topology. `0` is the time root. |
| `PARENT` | — | The parent's extended address, 16 hex characters as printed by the parent at boot. Required for `NODE_ID > 0`. |
| `BEACON` | `1` | `BEACON=0` disables Enhanced Beacons (the time root always beacons). |
| `SENDER` | off | Any non-empty value enables periodic acknowledged data traffic. |
| `DEST` | `PARENT` | Data destination, 16 hex characters. |
| `SEND_PERIOD_MS` | `10000` | Data traffic period. |
| `SCHEDULE` | `chain` | Cell layout: `chain`, `star` or `contention`. |
| `STATS_PERIOD_MS` | `0` (off) | Period of the neighbor-table dump (needs a log backend). |

Every node installs its own cells; the schedule advertised in the parent's
beacon is not adopted. Joined nodes send a periodic keep-alive to their time
source so they stay synchronized even without data traffic.

## Scenarios

Each node prints its extended address at boot
(`node 0: PARENT=aabbccddeeff0011`). Flash the next node with that value as
its `PARENT`.

### Super coordinator (time root)

Starts the PAN, beacons, and accepts associations from its children:

```sh
cargo embed --release --bin tsch-node \
    --features tsch,tsch-coordinator,defmt flash
```

### Coordinator + sender (relay node)

Joins and associates with its parent, beacons for its own children, accepts
their associations, and sends periodic data up-stream:

```sh
NODE_ID=1 PARENT=<addr of node 0> SENDER=1 cargo embed --release \
    --bin tsch-node --features tsch,tsch-coordinator,tsch-association,defmt flash
```

### Leaf + sender

Joins and associates, does not beacon, and sends periodic data to its parent:

```sh
NODE_ID=2 PARENT=<addr of node 1> BEACON=0 SENDER=1 cargo embed --release \
    --bin tsch-node --features tsch,tsch-association,defmt flash
```

### Leaf without association

Omit the `tsch-association` feature to synchronize to the network without
performing the association handshake (no short address is obtained):

```sh
NODE_ID=2 PARENT=<addr of node 1> BEACON=0 cargo embed --release \
    --bin tsch-node --features tsch,defmt flash
```

## Alternative schedules

- `SCHEDULE=chain` (default): linear chain, node `k` is the child of node
  `k - 1`, with a dedicated up cell per hop.
- `SCHEDULE=star`: every node is a direct child of the time root, each on its
  own dedicated up cell — no uplink contention.
- `SCHEDULE=contention`: all children share one common up cell, so uplink
  transmissions collide and exercise the shared-link CSMA-CA backoff.

For example, a star child:

```sh
SCHEDULE=star NODE_ID=3 PARENT=<addr of node 0> SENDER=1 cargo embed --release \
    --bin tsch-node --features tsch,tsch-association,defmt flash
```

The exact cell layout of each schedule is documented in
[`schedule.rs`](schedule.rs).
