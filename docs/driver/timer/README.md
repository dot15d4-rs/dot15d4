# Dot15d4's high-precision, low-energy, syntonized radio timer

As has been pointed out in the [radio driver specification](../radio/README.md),
dot15d4 requires a precise radio timer that deterministically interacts with
radio hardware to schedule radio tasks and measure timestamps.

In the following we collect use cases and requirements and derive a timer
specification from those.

## Task scheduling needs

In dot15d4 we deal with two types of tasks:

- best effort task: a task that must be executed as quickly as possible and
  starts at a deterministic offset to the end of the previous task
- timed task: a task that is required to start at a precise time

A timed task can only be scheduled once it can be guaranteed that no other best
effort or timed task will be able to precede it. To execute best effort tasks as
quickly as possible, they must be scheduled around timed tasks while respecting
the timing restrictions imposed by timed tasks.

Tasks are derived by upper layers from possibly concurrent MAC requests and then
forwarded to the driver. More than one driver task is usually derived from a
single MAC request and execution of driver tasks derived from distinct MAC
requests may interleave.

More than one task may be due at any given time which means that tasks need to
be ordered, scheduled and buffered before passed on to the driver. As the radio
can only execute a single task at a time, tasks may never overlap.

The "no-overlap" rule actually isn't so easy to implement because incoming
events may require dynamic updates to the schedule. Sometimes a scheduling
decision can only be taken a few microseconds before the actual task switch:

- The end of an rx window is not always known when it is scheduled. Initially it
  can be scheduled for an arbitrarily long time span (even "forever"). Incoming
  rx/tx tasks may then cancel the listening rx task (i.e. end the rx window).
- Ongoing packet reception may have to be cancelled in case an unexpectedly
  long packet is being received by a third party. If a timed tx packet or off
  task is scheduled while another packet is being received, we may have to
  interrupt reception. In the tx case we may have to report "CCA busy" in such a
  case.
- The decision whether a reception is to be followed by an ACK frame or not and
  which information elements need to be included in the response can only be
  taken after the frame header or even certain information elements have being
  received.
- Protocols will behave differently depending on whether the counterparty does
  or does not acknowledge an outbound frame.
- Best effort tasks may have to be dynamically shuffled around timed tasks when
  new MAC requests arrive that introduce additional timing restrictions.
- ...

This has important consequences for timer design:

- In some cases we need very short guard times. Timed tasks may have to be
  scheduled only a few microseconds ahead and ongoing timers may have to be
  interrupted. We therefore need to define a minimum scheduling time, i.e. the
  worst-case time it takes from sending the task to the message queue until it
  has been scheduled to the driver. This time needs to be measured on real
  hardware.
- The MAC service has to run a timeout to the earliest timed task (minus guard
  time) concurrently with request/response reception. When the timeout fires,
  the corresponding task can be scheduled to the driver. When a new, more urgent
  task arrives, the previous timeout must be canceled and a new timeout
  scheduled to the earlier expiration time. The timer is required to wake up the
  CPU in these cases.
- On the other hand, high precision radio tasks do not tolerate CPU
  intervention. The timer driver will have to schedule timed radio tasks via
  deterministic hardware events in such cases.
- Both use cases (waking the CPU and hardware events) need to share the same
  time base. This means that we need a common timer whose abstract API allows us
  to wake up OS tasks as well as schedule hardware tasks.
- Of course the API cannot be hardware/vendor-specific. We need to find
  vendor-agnostic abstractions for hardware tasks and events.
- It is the responsibility of the MAC service to correctly order tasks and
  ensure that tasks do not overlap. That's why we need to provide tasks with all
  the information required to calculate precise execution times, see "radio task
  requirements" below.

## Low-energy execution

On one hand, radio tasks have to be scheduled very precisely and at short term.
On the other hand there may be very long sleep times between radio tasks.

As radio devices may be battery powered and potentially have to run off a single
battery for years, we cannot neglect the impact of the timer on energy
consumption.

Low-energy devices usually have a low-power timer in the always-on power domain
that can run for long times with minimal energy consumption. Such timers are
running at a low frequency to save power. Uptimes of several years force us to
protect the sleep timer against overflow.

Scheduling radio tasks requires the best available timer resolution, though: The
more precisely the timer works, the less reception windows need to be widened to
accommodate for clock drift. As unnecessarily widening reception windows wastes
energy, we need a compromise between low energy consumption during sleep times
and high precision timing while the radio is active. As the high precision timer
should not be active for extended periods, overflow protection should not be
required for it.

To solve this dilemma, we fuse a low frequency sleep timer with a high-precision
wake timer. The high-precision timer will be synchronized to the sleep timer
whenever it is woken up and is then responsible to actually time radio tasks.

## Syntonization

**Syntonization** is the procedure required to tune clocks from different clock
sources to a common frequency. In other words: Syntonization intends to
eliminate (or at least minimize) clock drift.

**Synchronization** adds the requirement to agree upon a common epoch.

IEEE 802.15.4 protocols typically require syntonization but no synchronization.

Syntonization is more difficult to achieve than synchronization. Agreeing on a
common epoch once clocks have been syntonized is usually rather trivial.

Several IEEE 802.15.4 protocols require or at least benefit from syntonization
including but not limited to TSCH, CSL, BE PANs, etc.

Therefore we need to design a syntonization algorithm for dot15d4.

Syntonization is a well-known problem: Linux's PTP implementation is based on a
rather sophisticated PI(D) controller, Zephyr used a simple PI loop, too.
Contiki uses an average of a number of measurements. There are probably many
other options.

In a first step we focus on the syntonization requirements of TSCH.

TSCH establishes a hierarchical network of clocks that _syntonize_ to each
other: Some TSCH devices are defined as time sources for other TSCH devices.
Each TSCH device needs at least one time source neighbor.

Inside a single device this clock hierarchy is continued. A single device may
contain one or more local radio clocks that need to be _syntonized_ to the local
TSCH time source(s). Each radio peripheral inside a device is timed by exactly
one local radio clock.

In our case, a radio clock is again divided into a sleep clock and a
high-precision clock. Typically, the sleep clock is the master clock to which
the high-precision clock will _synchronize_ regularly.

### Restrictions on the TSCH syntonization algorithm

For TSCH we need to syntonize:

- one or several external "wall clocks", i.e. TSCH time sources
- at most one local "radio clock" per radio driver peripheral

Syntonization results in a shared notion of time, some sort of "master clock".
From the perspective of a single device, we call this (abstract) shared clock
_the_ "syntonized clock".

Conceptually, the syntonized clock SHALL be a monotonous and continuous ("gap
free") uptime clock. While clocks SHALL _syntonize_ their frequencies, they MAY
but don't have to be _synchronized_ to a common epoch ("0 value").

If we have a single TSCH time source, then that clock source's frequency is by
definition the frequency of the syntonized clock, i.e. the master frequency to
which local radio clocks need to syntonize.

If a single device listens to multiple TSCH time source neighbors, then some
(not yet specified) "average" clock frequency needs to be synthesized from those
clock sources. The result defines the local syntonized master clock.

To simplify radio driver implementation, the radio clock itself is modeled as an
independent local uptime clock whose physical time representation can be
converted to and from syntonized time. Syntonized time itself MAY but often will
not be represented physically. More often, devices will measure drift between
the physical clock and the syntonized clock from which they derive a set of
conversion parameters.

At the API level, the radio clock SHALL NOT expose physical clocks but an
abstract nanosecond resolution clock that can be converted to and from the
corresponding hardware clock(s).

Note that converting to and from nanosecond resolution is just a means to
abstract from the implementation details of the radio and timer hardware. More
specifically:

- to abstract from the internal distinction of a low-granularity sleep clock vs.
  a high-granularity high-precision clock,
- to abstract from the granularity differences between distinct radio and/or
  timer hardware,
- to represent offsets and drift between different clocks independently of
  hardware details.

Abstracting from hardware details means that we don't have to make the radio
driver and radio timer APIs generic over hardware clock frequencies. This
considerably simplifies the API.

Local radio clocks are typically assembled from a low frequency sleep clock and
a high-precision timer. As those usually run from physically distinct
oscillators, their notion of time needs to be fused, too, to represent a single
radio clock at the API level.

Conceptually, the fused radio clock SHALL be conceived as running at its
high-precision clock's frequency all the time. This gives us well-defined,
uniquely enumerable, gap-free integer radio clock "uptime ticks". These discrete
uptime ticks will then be deterministically and uniquely "labelled" in terms of
"nanosecond uptime ticks":

- A given radio clock tick SHALL always be converted to the exact same
  "nanosecond tick" value, i.e. we select a unique "nanosecond tick" per unique
  radio clock tick which makes the tick-to-ns conversion "one-to-one" but not
  "onto".
- Conversely, "nanosecond ticks" SHALL be "rounded" to well-defined radio clock
  ticks, i.e. the ns-to-tick conversion is "onto" but not "one-to-one".

Note: This assumes that all radio clocks run at frequencies lower than 1GHz. We
chose nanosecond resolution as a compromise between the ability to represent a
wide range of existing radio clock hardware and efficient representation in 64
bits without overflow (~584 years). This assumption will not hold for UWB PHYs.
Their nominal radio clock frequency is > 1GHz. Once we start working with UWB
hardware, we'll need more bits to represent a non-overflowing uptime clock.

The internal clock fusion algorithm SHALL ensure, that the conversion of sleep
timer ticks into a high-frequency timer representation is similarly
deterministic, i.e. a radio clock tick can equivalently be uniquely labeled as a
tuple `(s, h)` where `s` represents a sleep timer tick and `h` some
high-precision timer offset. Note that due to drift between the two clocks, the
number of valid high-precision timer offsets per sleep timer interval can vary
for individual sleep timer ticks.

The MAC service calculates timestamps in terms of the syntonized clock. For now,
emitting timestamps at nanosecond granularity will suffice for all practical
reasons. This is not a conceptual limitation, though. Syntonized time can be
calculated and represented at an arbitrary level of precision.

If we speak of syntonized nanosecond clock "ticks" below, then this is just the
current approximation, "good enough" to implement TSCH.

A syntonization function SHALL be a pair of conversion functions between the
syntonized clock and a given radio clock: They take an arbitrary tick from one
of the two clocks and transform it into a tick of the other clock:

Conversion from the syntonized clock to the radio clock:

- surjective but not injective: Every syntonized clock tick SHALL be "rounded"
  to a unique radio clock tick.
- monotonic: Given syntonized clock ticks `n1`, `n2` with `r1`, `r2` as their
  corresponding radio clock ticks, `n1 > n2 => r1 >= r2` SHALL hold.

Conversion from the radio clock to the syntonized clock:

- injective but not surjective: Every radio clock tick SHALL be represented by a
  unique syntonized clock tick.
- strictly monotonic: Given radio clock ticks `r1`, `r2` with `n1`, `n2` as their
  corresponding syntonized clock ticks, `r1 > r2 => n1 > n2` SHALL hold.

More intuitively: Conversion from syntonized time to radio clock ticks can be
visualized as a monotonously increasing step function. It assigns a unique
syntonized clock increment to each radio clock tick. I.e. to compensate for
clock drift, extra syntonization "nanoseconds" can be injected or removed from a
radio tick-to-tick increment. This step function SHALL never become so "flat" or
"decreasing" as to assign a negative or zero increment to a radio clock tick. It
also SHALL NOT become so "steep" as to "jump over" a clock tick.

The syntonization function is further restricted by discrete "syntonization
events" derived from frames received from a time source neighbor:

- We know the "expected reception timestamp" in terms of the syntonized clock.
- We measure the corresponding "actual timestamp" in terms of the radio clock.

Syntonization events fix points (i.e. ns/tick tuples) on the conversion
functions. (Or they are fed into something more sophisticated like a PI
controller...)

A syntonization algorithm defines an interpolation between points fixed by
syntonization events under the monotonicity and injection/surjection constraints
outlined above while at the same time trying to minimize the interpolation error
as compared to the "real" drift between the time source neighbor's radio clock
and the local radio clock:

- Additional syntonization events SHALL never lead to a change in the
  syntonization function for radio ticks that lie in the past or within the
  scheduling guard time (i.e. ticks that we may already have converted to).
- Additional syntonization events MAY change the ns-to-tick mapping of radio
  ticks beyond the scheduling guard time.
- We SHALL never store any converted radio tick beyond the scheduling guard
  time.

To make this practical, we use syntonized time representation in driver service
requests as these don't have to be re-calculated after syntonization events.

We delay the actual syntonization-clock-to-radio-tick conversion (and hence
scheduling of radio tasks) as much as possible such that conversions will rarely
be obsoleted in practice by inbound syntonization events.

However, no matter how much we delay conversion, this introduces a race between
pre-calculated timestamps for future operations and measured timestamps for past
reception that may change the syntonization parameters for previously calculated
timestamps.

Currently we'll live with the error introduced by that race. As time source
relationships are hierarchical and directed, it should only slightly delay
syntonization across the network but not cause oscillations or other
misbehavior.

# Hard Realtime Signalling and Timestamping

Most timer APIs will allow client tasks to schedule a timeout that will then
wake the CPU and return control to the client task at some time "shortly after"
the deadline. This is true for RTIC, embassy, Zephyr, Linux, etc.

Typical generic timer APIs and their implementations rely on a single hardware
counter peripheral. Neither their API nor their implementations allow the
required combination of a sleep-timer and a high-precision timer. Again this is
true at least for the timer implementations mentioned in the previous paragraph.

Timing with CPU intervention is ok to serve upper layer MAC/driver tasks which
run off the CPU anyway, but it is _not_ sufficient to time the radio. As already
pointed out in the [radio driver architecture section](../radio/README.md), we
cannot afford CPU intervention when it comes to standard-compliant scheduling of
radio tasks and event timestamping. Working off a single timer peripheral is not
efficient enough as has been pointed out above.

Typical radio drivers will therefore not rely on vendor-independent timer APIs
but implement their own vendor-specific hybrid timing scheme integrated into the
radio driver itself (if they provide high-precision timing and timestamps all).
This is true for vendor-specific IEEE 802.15.2 stacks like the ones from NXP or
TI as well as Nordic's open source stack in Zephyr.

As we want to make the job of future community contributors and driver
maintainers as easy as possible we're not satisfied with this approach. We want
a vendor-agnostic API that captures difficult design trade-offs in Rust traits
and helper functions making proper timer and radio driver design less difficult.

Exposing a timer API also has the advantage, that it can be used outside the
radio driver. It can be used in upper protocol layers and even more importantly:
we'll be able to provide a sub-microsecond precision network-global "wall clock"
to applications (independently of NTP/PTP/...) which is an important asset in
many use cases.

We therefore need an abstraction of hardware signals and events that can be
scheduled/measured through a vendor-independent API without any CPU intervention
on timing-sensitive execution paths. We want this API to catch as many
implementation problems as possible at compile time. Therefore our API will
again use static typing approaches like typestate, RAII, linear types, etc. to
enforce correctness and ease development both, on the client and driver
implementation sides.

# Requirements

The following requirements can be derived from the above use cases and restrictions.

## application domain

- high-precision scheduling of (hardware) signals driving a single radio task at
  a time
- capturing of high-precision timestamps for radio (hardware) events
- waking up the MAC and/or driver service tasks at a predefined (coarse)
  sleep-timer tick
- high-precision auxiliary scheduling and measurement (e.g. GPIO test signals
  and trace events)
- optionally: waking up arbitrary OS tasks based on a shared notion of global
  syntonized time (global "wall clock")

The timer can either be multiplexed via "hardware channels" or an additional
timer queue. As long as we only support timing of the MAC/driver services, radio
signals/events and GPIO measurements, available hardware channels suffice. If we
want to support waking up arbitrary OS tasks, then we'll need a timer queue.

## syntonization domain

- long-term timeouts must be given in syntonized time: no need to re-calculate
  timeouts after syntonization events
- pluggable syntonization algorithm: e.g. average, PID, ...
- syntonized time must be monotonic and gap-free as described above

One complication at the syntonization-to-counter interface is that according to
the standard, the radio timer should count precise (fractions of) symbol periods
or ranging scheduling time units (RSTUs). To support syntonization across
devices, we need an absolute/global timescale (e.g. measured in nanoseconds).
Also, most non-ranging (non-ERDEV) hardware does not provide radio counters at
symbol resolution. We currently use specifically tuned integer fractions to
compromise between conversion speed and precision.

## counter domain

- hybrid sleep and high-precision timer combining low energy consumption, high
  precision and low jitter with minimal synchronization overhead and guard times
- unless required by the application, the HF crystal should be released while
  the timer is in sleep mode
- overflow protection: capable to measure absolute uptime for at least 20 years
- at least two independent sleep-timer compare channels (see application domain
  above) in addition to up to three independently scheduled hardware events
- at least two independent high-precision event timestamp capturing channels
  working in parallel with the scheduling channels
- a test mode with GPIO signals and events for debugging and tuning

## timer queue

The timer queue is optional unless we want to support waking up arbitrary OS
tasks, see application domain above.

- the timer queue would ideally be intrusive (i.e. support an arbitrary number
  of timers w/o pre-allocation), const capacity is ok, too
- the timer queue should be interrupt/preemption-safe, i.e. synchronized

### preliminary evaluation of some existing queue implementations

- [embassy queue (integrated)](https://github.com/embassy-rs/embassy/blob/main/embassy-time-queue-utils/src/queue_integrated.rs): tightly coupled to embassy executor
- [embassy queue (constant)](https://github.com/embassy-rs/embassy/blob/main/embassy-time-queue-utils/src/queue_generic.rs): constant capacity, built around heapless::Vec, represents time as u64, only depends on core and heapless, requires &mut self, i.e. not synchronized
- rtic queue: based on own [linked list](https://github.com/rtic-rs/rtic/blob/master/rtic-time/src/linked_list.rs), only requires &self for mutation (multi producer), heavily relies on critical sections for synchronization
- [cordyceps sorted list](https://docs.rs/cordyceps/latest/cordyceps/struct.SortedList.html): requires &mut self
- [heapless sorted list](https://docs.rs/heapless/latest/heapless/sorted_linked_list/struct.SortedLinkedList.html): requires &mut self
- [lilos list](https://docs.rs/lilos-list/0.1.0/lilos_list/struct.List.html): explicitly intended for timer queues, can be shared across tasks via Pin<&List>, not synchronized/no locking, can only be used with futures
- crossbeam: requires std/alloc
- Harri's sorted linked list: MPSC, lock-free, requires marked pointers, complex
- lock-free MPSC queue + non-synchronized sorted linked list exclusively owned
  by the timer interrupt: requires a separate message to cancel timers

The last option would work as follows:

- insertion of timers (with a waker and optionally an associated hardware
  signal) and timer cancellations into a message queue from multiple producers,
  including at higher priorities than the target (timer) interrupt
- after insertion, the target interrupt is pended
- whenever active, the target interrupt drains the queue, inserts timers locally
  into a sorted list and (re-)schedules head

# Driver Service Request and Radio Task Requirements

The following timing-related fields will have to be communicated from the MAC
service to drivers via the driver service.

## Reception (including Filtering)

Rx-related fields

- startOfWindowInstant: the instant at which the receiver must be ready
- windowDuration (aka "last frame start"): the duration after which the receiver
  must be disabled unless a frame was recognized
- maxRxPpduLength: the max expected length, if the received frame is larger, the
  frame will be discarded and the receiver disabled before the end of the frame
  (this allows us to define precise timings for Rx Windows that cannot be
  manipulated by third-party frames)
- guardTime: a drift-dependent guard time to be added to the start and end of an
  Rx Window (only required by higher layers, the radio driver expects guard
  times to be "priced in" already)

Tx/ACK-related fields

- ifsDuration: the IFS duration may either be non-standard or protocol specific
  (see IFS and TSCH timing spec)
- ackRepr: allows us to calculate the Tx duration of the ACK packet for precise
  window calculations

The returned frame needs a precise radio clock timestamp for its RMARKER.
Depending on the abstraction layer, this may be represented as a radio clock
tick or in syntonized time.

## Transmission

Tx-related fields

- startOfFrameInstant

Rx/ACK-related fields

- ifsDuration: see Rx
- ackRepr: see Rx
- guardTime: see Rx

The task response needs to include a precise radio clock timestamp for the
RMARKER of the sent package (at least for best-effort tasks). Depending on the
abstraction layer, this may be represented as a radio clock tick or in
syntonized time.

## Mapping to radio tasks

On the radio task side, timestamp fields are mapped to at/start arguments of the
driver API. The inline documentation specifies the details already. Timestamp
fields in radio tasks are measured in terms of the radio clock.

## Open questions

It's not yet clear in what "units" (driver service requests) the above fields
need to be communicated from the MAC service to the driver service.
