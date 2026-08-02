# V29 temporal cross-formation edge suspension

## Motivation

V28 identifies a repeatable direction but not a strong action.  Reducing one
reference cross-edge weight from 0.05 to 0.025 improves tracking,
cardinality, and H=3 consensus for three of four formations, yet the best
mean E-OSPA gain is only 0.86%.  The same screen also falsifies one-step
posterior disagreement as a hard recursive guard: every damping action has
higher predicted one-step disagreement, while three have 2.32%--2.95%
better realized window consensus.

V29 makes the smallest structural change that amplifies the validated
mechanism.  If a reference cross-formation payload is currently harmful to
label existence, the sender is omitted for one step rather than assigned a
small positive weight.  The dropped 0.05 weight is returned to receiver
self weight; it is not transferred to another neighbor.  The next two steps
use the registered reference route.

## Action and scale

There is one binary suspension decision for each formation's registered
incoming cross edge.  For `F` formations, the diagnostic oracle contains
`2^F` subsets including reference.  This is 16 actions for M24, 64 for X36,
and 256 for X48.  The deployable representation remains `F` per-formation
scores: a later GNN can predict each edge's value and a deterministic
projector can select a safe subset.  Exact enumeration is used only to
establish headroom and audit interactions at the current scales.

Every suspended edge removes one attempted message at the intervention
step.  Across the registered H=3 sequence, suspending `k` edges saves
`k/(3 M_ref)` of directed messages, where `M_ref` is the per-step reference
message count.  Actual payload-byte saving is still measured because sender
posteriors have different sizes.

## Safety model

The candidate is constructed from the current posterior, physical graph,
current link probabilities, and the two prior selected topologies.  It uses
no truth or future outcome.  A subset is executable only if:

- reference-weighted existence-retention risk is at most 0.01;
- every formation's mean expected-cardinality change is at least -0.05;
- every reference-supported label retains at least 80% of its expected
  existence probability;
- no reference label expected above 0.5 falls below 0.5;
- all fusion rows remain nonnegative and sum to one;
- selected messages are physical and no more numerous than reference;
- every sensor-level and formation-level rolling three-step window is
  strongly connected for the sequence `{suspension, reference, reference}`.

Instantaneous strong connectivity is deliberately not required.  A one-step
drop can disconnect the formation-level support at that instant, but the
rolling window still carries every formation's information.  This is the
temporal connectivity model the earlier sparse-topology story needed but
did not enforce precisely.

One-step posterior disagreement is reported as a diagnostic only.  It is
not used as a hard gate because v27 and v28 provide direct counterexamples
to treating it as a recursive correctness or consensus certificate.

## Frozen outcome gate

The first preflight and H=3 screen reuse only M24 seed 211 at `t=72`.  The
screen executes reference and every current-posterior-safe subset with
common random numbers.  A strong action requires:

- at least 2% mean E-OSPA gain;
- nonnegative minimum-formation and worst-sensor gains;
- nonnegative H-window consensus gain;
- nonnegative attempted-byte saving;
- selected rolling-B3 passage at all three steps.

If the primary screen has no strong action, v29 stops without opening
another state.  If it passes, the other preregistered opened M24 states may
be screened.  GNN training still requires strong headroom on at least three
of four states.  X36, X48, and reserved validation seeds remain sealed.

## Paper-level contribution if successful

The method would combine a KLA-specific conflict bound with temporal graph
control:

1. the `eta`-driven existence-retention guard prevents destructive fusion;
2. rolling connectivity guarantees delayed information flow without
   requiring every instantaneous graph to be strongly connected;
3. a learned value model predicts which already-safe suspensions improve
   multi-step tracking and consensus;
4. dropping rather than reweighting messages produces direct communication
   savings.

This is both an estimation-aware topology intervention and a scalable
data-driven decision problem.  The learning component is not introduced
until the exact subset oracle demonstrates repeatable multi-state headroom.
