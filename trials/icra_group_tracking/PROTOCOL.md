# ICRA dynamic-group tracking pilot v1

Frozen before tracking results, 2026-09-07. User authorized both mechanism
scenes and the matched-communication comparison. Work on `codex/icra`.

## Scope and inputs

- Eight mobile sensors, 120 frames at 0.5 s, two deterministic smooth 2-D
  trajectories: `split_rejoin` and `boundary_churn`. Two teams of four move
  apart and back, or repeatedly cross the radio boundary. These are synthetic
  mechanisms, not WHALES recordings. No motion controller is optimized.
- Radio range 60 m. Within-team rectangles deform smoothly, so nearest
  neighbors can change without sensor identities changing. Global coordinates
  and a shared clock are assumed known. Sensor speed/acceleration and actual
  component changes are audited before evaluating tracking.
- Three paired seeds: 2701, 2702, 2703. Each seed generates target initial
  offsets, process perturbations, observations and directed packet uniforms
  once, shared across all four arms and both scenario comparisons where
  applicable. Three seeds are a screen, not a significance/generalization test.
- Three targets active throughout, drawn around three of four fixed known
  birth regions. The estimator receives all four broad priors (r=0.7,
  position std=12 m, velocity std=0.5 m/s), only at frame 1. It is not given
  realized initial states, active region indices, or true target count.
  This tests continued tracking of initially born targets, not arbitrary
  target arrivals or measurement-driven births. Survival probability 0.995.
- Global Cartesian measurements, std=1 m, pD=0.9 within a 90 m circular FoV,
  zero pD outside. Circular FoV deliberately isolates neighbor/group changes
  before adding heading limitations. Poisson clutter mean 1, uniform over
  fixed world rectangle [-130,130] x [-80,120] m. No detections labeled by ID.
- Existing LMB prediction/update, LBP association and mixture-aware KLA
  primitives; maximum eight GM components per label, support-renormalized
  missing-label semantics, one synchronous posterior-exchange round/frame.
  Extraction: MAP cardinality, highest-weight Gaussian per selected label.
  All arms share the same estimator settings. No stale-message cache/replay;
  node posteriors persist through group changes. No consistency guarantee is
  inferred from this simple treatment of reconnection.

## Four arms

1. `local`: independent local LMB, no communication.
2. `tree`: recompute distance-minimal bidirectional spanning tree inside each
   current physical component.
3. `dynamic`: rebuild groups of at most three each frame; smallest permanent
   UID seeds a group, add its nearest compatible nodes while retaining a
   physical clique. Build a distance-minimal tree inside each group and then
   connect groups by minimum-distance physical edges without cycles.
4. `stable`: retain each previous group if its clique is physically valid.
   Immediately split invalid groups with the same deterministic rule. Merge
   groups (max size three, physical clique) only when every cross link has
   existed for at least three consecutive frames. Initialize with the same
   grouping as `dynamic`. Within-/between-group trees still recompute from
   current geometry: stability acts on membership, not an extra edge-memory
   heuristic. No target outcome enters any topology decision.

For both grouping arms, group-level connections restore a tree in every
physical component even before membership merges. The merge wait therefore
does not deliberately block a newly available physical reconnection.
Singletons/pairs are legal; all selected edges must exist physically.
This is a simple candidate, not a claim of a novel clustering algorithm.

## Matched communication

All three communicating arms use exactly 2(N-C_t) directed transmissions
per frame, where C_t is the number of physical components. Each carries one
8192-byte padded posterior packet. The implementation serializes header,
label, existence, mixture weights, means and full covariances, asserts it
fits, reconstructs the receiver input from bytes, then discards padding.
Packets carry sender and frame IDs; node/target IDs are never derived from
current group numbers. Independent directed loss probability 0.1; identical
pre-generated (receiver,sender,frame) uniforms across arms. Failed attempts
are charged. Metropolis weights are computed on the scheduled undirected
tree, then each missing neighbor's weight returns to the receiver's self
weight. No expected-delivery approximation or instantaneous multi-hop fusion.

The prototype uses centralized geometry coordination, with a modeled
reliable out-of-band control channel. Each frame charges each node 64 bytes
uplink (pose/status) and 64 bytes downlink (group/neighbor configuration),
the same for all communicating arms. This assumption is explicit: a lossy
distributed group-agreement protocol is not implemented. Log unpadded
serialized payload bytes and control bytes separately, as well as total
padded attempted wire bytes. These are simulated protocol byte counts, not
measurements from deployed radios. Padding fixes cost for the first causal
comparison; it does not establish bandwidth efficiency without padding.

## Metrics and decision

- Primary: full-episode mean position OSPA, p=2/cutoff=30 m, and absolute
  count error over every node/frame; also per-frame worst-node and node/frame
  p90 OSPA. Keep all three targets in primary evaluation during separation.
- Record estimates/labels, pre-fusion and post-fusion count/OSPA, conditional
  assignment RMSE with matched support, attempted/delivered messages, padded
  and unpadded bytes, topology runtime, filter runtime, membership pair edits
  (invariant to group labels), and topology edge edits.
- Plot post-reconnection mean/worst-node OSPA. Summarize first 10 frames
  after each physical full-network reconnection. Recovery time is first
  five-frame window with mean OSPA <=5 m, censored at the next disconnection
  or episode end. A trajectory already satisfying the threshold at reunion
  has zero delay; do not force an artificial transient.
- Audit truth visibility for interpretation only; never use it to select
  groups, remove hard targets, or reset priors. Verify geometry and identity,
  packet serialization, graph support/connectivity and matched total budgets.
- No parameter search in this pilot. Continue if grouping stability gives
  repeatable tracking/recovery benefits beyond the plain time-varying tree
  at the matched cost. If only switching falls without task benefit, or only
  a failed fixed-group baseline is beaten, report a weak/negative signal and
  recommend narrowing/changing direction. Do not claim publication readiness.
