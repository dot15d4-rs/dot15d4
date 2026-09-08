# TSCH Example

`tsch-node` is a single binary that can play every role of a small TSCH
network. Role, topology and traffic are fixed at compile time through cargo
features and environment variables (read with `option_env!`, so set them on
the `cargo` command line). The cell layouts and the network entry procedure
live in [`schedule.rs`](schedule.rs), the log grammar in [`val.rs`](val.rs).

All commands assume `cd examples/nrf52840`.

## Configuration

Cargo features:

| Feature | Meaning |
|---|---|
| `tsch` | Required by this example. |
| `tsch-coordinator` | Accept association requests from child nodes. |
| `tsch-association` | Perform the MLME association handshake with the parent after joining, and read macShortAddress back. The parent must be built with `tsch-coordinator`. |
| `defmt` (or `log`) | RTT log output. Needed for every `val` line, including the node's address at boot. |
| `tsch-log` | Trace the TSCH state machine (slot scheduling, link selection, backoff, frame outcomes). |
| `sync-error-log` | Log the per-resync synchronization error (`src=frame` / `src=ack`), plus an observation-only error against any neighbor that is not the time source (`src=peer`, nothing applied). Positive means the local clock is ahead. Requires `defmt`. |
| `no-tsch-ack-sync`, `no-tsch-frame-sync`, `no-tsch-drift-adjustment` | Disable individual synchronization mechanisms. |

Node and traffic variables (compile time):

| Variable | Default | Meaning |
|---|---|---|
| `NODE_ID` | `0` | Position in the topology. `0` is the time root. |
| `PARENT` | — | The parent's extended address, 16 hex characters as printed by the parent at boot. Only beacons from that address are accepted and only its clock is followed. Required by the static layouts for `NODE_ID > 0`; optional, but recommended, with `SCHEDULE=learn`. |
| `BEACON` | `1` | `BEACON=0` disables Enhanced Beacons (the time root always beacons). |
| `SENDER` | `0` | `SENDER=1` enables periodic acknowledged data traffic. |
| `DEST` | `PARENT` | Data destination, 16 hex characters. Falls back to the coordinator the node joined. |
| `SEND_PERIOD_MS` | `10000` | Data traffic period. |
| `STATS_PERIOD_MS` | `0` (off) | Period of the neighbor-table dump (`val neighbor` lines). |

Network variables, read by [`schedule.rs`](schedule.rs):

| Variable | Default | Meaning |
|---|---|---|
| `SCHEDULE` | `chain` | Cell layout: `minimal`, `chain`, `star`, `contention`, `mesh` or `learn`. |
| `BEACON_PERIOD_S` | `10` | Enhanced Beacon period of every beaconing node. |
| `BEACON_JITTER_MS` | `0` | Random `+/-` jitter on that period. Set it when two neighbours beacon on the same cadence in the same shared cell, otherwise their beacons can collide round after round. |
| `CHANNEL` | — | Pin the network to one IEEE channel (11..=26): the time root advertises a hopping sequence of one entry and every node scans there, so a sniffer parked on that channel captures every frame. Unset means the default hopping sequence, and scanning on channel 26. |
| `NODE_COUNT` | — | How many nodes take part. Required by `mesh`; used by `star` to size the root's listen cells. |
| `SLOTFRAME_SIZE` | per layout | Override the slotframe length, in timeslots. Two layouts compared against each other need the same value, otherwise their cells recur at different rates. |
| `PARENT_ID` | — | `mesh` only: the parent's `NODE_ID`. It does not change the cells, it only makes the node abort at boot when the ids are not in synchronization order. |

Stack constants are set the same way, with a `DOT15D4_` prefix, for instance
`DOT15D4_MAC_TSCH_KEEP_ALIVE_PERIOD=400` (400 timeslots, 4 s with the default
template) or `DOT15D4_MAC_TSCH_MAX_LINKS=12`.

With a static layout (`minimal`, `chain`, `star`, `contention`, `mesh`) every
node installs its own cells and ignores the schedule advertised by its parent.
With `learn` the node installs no cells and copies the slotframe and the links
from the beacon it joins through. In both cases the ASN, the timeslot template
and the hopping sequence come from that beacon.

## Log grammar

With a log backend, the node prints one line per validation event, next to the
`tsch <event> key=value ...` lines of the stack:

```text
val boot node=0 role=root addr=88c4b2aeee4fae4e ms=12
val pan started ms=1043
val scan done found=1 ms=4210
val join ok parent=88c4b2aeee4fae4e pan=beef ms=4907
val assoc ok short=2a83 ms=5311
val assoc peer dev=3b8be9e6da5f2a83 short=2a83 ms=5402
val tx seq=17 ok=1 ms=515002
val rx seq=17 src=3b8be9e6da5f2a83 ms=515003
```

`ms` is the node's uptime. `val tx`/`val rx` carry the 4-byte counter the data
payload holds, so delivery can be counted end to end without reading a MAC
counter. See [`val.rs`](val.rs) for the full list.

## Scenarios

Each node prints its extended address at boot (`val boot ... addr=...`). Flash
the next node with that value as its `PARENT`.

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

Omit the `tsch-association` feature to synchronize without the association
handshake (no short address is obtained).

### Functional validation (V1)

A three-node line on the minimal 6TiSCH schedule, where the joining nodes take
their cells from the network. The root:

```sh
SCHEDULE=minimal CHANNEL=20 BEACON_JITTER_MS=500 \
    DOT15D4_MAC_TSCH_KEEP_ALIVE_PERIOD=400 \
    cargo embed --release --bin tsch-node \
    --features tsch,tsch-coordinator,tsch-log,sync-error-log,defmt flash
```

The relay, pinned to the root, and the leaf, pinned to the relay (only
`NODE_ID`, `PARENT` and `BEACON` differ):

```sh
SCHEDULE=learn NODE_ID=1 PARENT=<addr of node 0> SENDER=1 SEND_PERIOD_MS=30000 \
    CHANNEL=20 BEACON_JITTER_MS=500 DOT15D4_MAC_TSCH_KEEP_ALIVE_PERIOD=400 \
    cargo embed --release --bin tsch-node \
    --features tsch,tsch-coordinator,tsch-association,tsch-log,sync-error-log,defmt flash
```

A keep-alive period below the beacon period is what makes keep-alives fire
between beacons, so every Enhanced ACK exercises acknowledgment-based
synchronization.

### Shared-cell contention (V2)

Two children share one up cell with the root, so their uplink frames collide
and the shared-link CSMA-CA backoff runs. Root:

```sh
SCHEDULE=contention CHANNEL=20 cargo embed --release --bin tsch-node \
    --features tsch,tsch-coordinator,tsch-log,sync-error-log,defmt flash
```

Each child, with a period short enough for the frames to meet in the same
cell:

```sh
SCHEDULE=contention NODE_ID=1 PARENT=<addr of node 0> BEACON=0 \
    SENDER=1 SEND_PERIOD_MS=250 CHANNEL=20 \
    cargo embed --release --bin tsch-node \
    --features tsch,tsch-association,tsch-log,sync-error-log,defmt flash
```

The control arm is the same traffic on dedicated cells: replace
`SCHEDULE=contention` with `SCHEDULE=star` everywhere, add `SLOTFRAME_SIZE=8`
so both arms keep the same cell cadence, and `NODE_COUNT=3` on the root.

## Alternative schedules

- `SCHEDULE=minimal`: the minimal 6TiSCH schedule of RFC 8180. One shared
  advertising cell in timeslot 0 of an 11-slot slotframe carries beacons,
  association, keep-alives and data, in both directions. The layout does not
  depend on `NODE_ID`, so the root advertises it and any node can take it over
  with `SCHEDULE=learn`. This is also what a Contiki-NG node expects.
- `SCHEDULE=chain` (default): linear chain, node `k` is the child of node
  `k - 1`, with a dedicated up cell per hop.
- `SCHEDULE=star`: every node is a direct child of the time root, each on its
  own up cell, so there is no uplink contention.
- `SCHEDULE=contention`: all children share one up cell, so uplink
  transmissions collide and exercise the shared-link CSMA-CA backoff.
- `SCHEDULE=mesh`: the cells impose no topology. Every node transmits in one
  advertising cell of its own (timeslot `NODE_ID`) and listens in every other
  node's, so everybody hears everybody. The topology comes from `PARENT` alone:
  the join policy drops every beacon from anyone else, and the stack never
  re-selects a time source, so a node still corrects its clock from its parent
  only. Every *other* beacon falls through to the `src=peer` log, so one
  transmission is measured by all other nodes at once and a run yields the full
  pairwise error matrix. Needs `NODE_COUNT`, and costs one link per node: above
  five nodes, raise `DOT15D4_MAC_TSCH_MAX_LINKS`.
- `SCHEDULE=learn`: no static cells at all; the slotframe and the links come
  from the beacon the node joins through.

A star child:

```sh
SCHEDULE=star NODE_ID=3 PARENT=<addr of node 0> SENDER=1 cargo embed --release \
    --bin tsch-node --features tsch,tsch-association,defmt flash
```

A mesh node; only `NODE_ID` and `PARENT` change between them:

```sh
SCHEDULE=mesh NODE_COUNT=7 NODE_ID=3 PARENT=<addr of node 2> PARENT_ID=2 \
    DOT15D4_MAC_TSCH_MAX_LINKS=8 \
    cargo embed --release --bin tsch-node --features tsch,sync-error-log flash
```

The exact cell layout of each schedule is documented in
[`schedule.rs`](schedule.rs).
