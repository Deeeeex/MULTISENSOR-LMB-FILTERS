# V46 paired tracking: outcome-opening design

## Claim to test

The primary empirical claim is a communication--estimation trade-off, not an
accuracy-improvement claim.  Relative to the repaired V46 full-communication
reference, the synchronized B4 arm should:

1. reduce attempted directed messages by exactly 37.5% over every aligned
   four-step cycle;
2. reduce attempted posterior bytes materially under the same always-heavy
   payload semantics; and
3. remain non-inferior in tracking error and distributed consensus on both
   M24 and X36.

The 37.5% message reduction follows from the frozen schedule: every receiver
attempts its dominant input on all four pages and its residual input on only
the first page.  The candidate therefore attempts five inputs per receiver
per cycle, whereas the reference attempts eight.  This does not imply 37.5%
byte reduction because sender posterior sizes can differ across edges and
time.  Attempted and delivered bytes must be measured separately.

Tracking accuracy is not part of the V46 structural theorem.  A candidate
that saves communication but exceeds the frozen non-inferiority margins is a
negative result and must not be described as successful.

## Evidence order

Outcome opening is split into four irreversible stages.

### Stage 0: structural prerequisites

The following artifacts must be committed and independently self-verified:

- the 32-case, 160-step causal-repair preflight on the registered M24/X36
  radial, convoy, relay, and crossing geometries;
- the deterministic eight-step real-filter smoke using physical-UID-keyed
  paired delivery uniforms; and
- a post-hoc audit that binds both artifacts to their executable source.

No tracking permit exists before all three checks pass.

### Stage 1: source-only case sealing

The development seed is fixed as `1009`.  The unopened confirmation seeds are
fixed as `[1013, 1019, 1021, 1033]`.  The originally registered fourth seed,
`1031`, failed the source-only geometry contract for
`x36-formation-fov-relay`: its minimum sensor--target separation was
`29.956118945301 m`, below the frozen `30 m` floor.  No filter or tracking
metric had been run.  It was replaced by `1033`, the smallest unused prime
greater than `1031`, after a geometry-only check passed all eight registered
scene styles.  This replacement rule may not inspect tracking outcomes, and
seed choice may not change after any V46 tracking metric is opened.

The registered case order is:

1. `m24-formation-fov`
2. `m24-formation-fov-convoy`
3. `m24-formation-fov-relay`
4. `m24-formation-fov-crossing`
5. `x36-formation-fov`
6. `x36-formation-fov-convoy`
7. `x36-formation-fov-relay`
8. `x36-formation-fov-crossing`

For each case, the source-only sealer generates the deterministic model,
measurements, truth, sensor trajectories, physical graph, and link-loss
schedule.  It replaces array-index random delivery draws with a tensor keyed
by physical sender UID, physical receiver UID, and time.  It records only
canonical hashes and structural metadata; it does not run either filter or
compute any truth-derived metric.

The generator-side model is not itself a legal filter input.  It contains
target trajectories and other metadata needed to reproduce the synthetic
scene.  A separate exact-schema sanitizer constructs the runtime model from a
field whitelist.  The filter-visible boundary retains the dynamic and sensing
models, the registered birth prior, the sensor-quality and FOV schedules, and
the physical-identity metadata required by V46.  It excludes generated target
trajectories, ground-truth RFS objects, target-formation metadata, duplicated
sensor-trajectory copies, generator validation results, and any state estimate
or tracking score.  Sensor trajectories are supplied once as an explicit
runtime input.

The registered birth prior is an experimental assumption, not a hidden target
trajectory.  It fixes possible birth locations, times, probabilities, means,
and covariances before either arm runs; posterior history buffers are reset.
Both arms receive exactly the same prior.  Because the synthetic prior is
centered on the configured birth locations, the paper must disclose this
assumption and include a later sensitivity check with wider covariance or
perturbed birth locations.  The input boundary therefore supports the narrow
statement that no realized target trajectory is passed to the filter; it is
not a claim that deployment knows exact births.

Two different hashes are retained deliberately.  The full source fingerprint
commits to every reproducibility object, including hashes of truth used only
for later scoring.  The runtime-v3 projection commits only to the sanitized
model, measurements, sensor trajectories, neighbor map, communication inputs,
physical identity, and delivery tensor that can cross the filter boundary.  A
filter permit may bind only the runtime-v3 projection.  Passing the full source
fingerprint or its envelope into the filter would reintroduce a transitive
truth commitment and is forbidden.

Radial, convoy, and relay scenes form the primary matrix.  Crossing remains a
named stress matrix because its V5 geometry contract is deliberately not a
formal validation scene.  A stress result cannot rescue a failed primary
gate, and a stress failure must be reported separately rather than hidden in
an aggregate.

### Stage 1 authority chain

The evidence and authorization objects form a one-way dependency graph:

`structural protocol -> core source registry -> source/runtime fingerprints -> offline source-provenance registry`.

The execution branch is separate:

`core case + independently accepted runtime-v3 case entry -> phase-specific permit -> filter execution`.

No object may point backward in this chain.  In particular, fingerprints do
not contain a freeze-registry hash or permit handle, and the freeze builder
cannot authorize its own output.  The complete 40-case discovery is first
published as a non-authorizing artifact from a clean, stable commit.  A later
independent commit may hard-code and audit its hashes in an offline provenance
registry, which must still declare filter execution and tracking scoring false.
Because that registry commits to full source and source-envelope hashes, it is
not eligible as a permit dependency.  The execution authority must be a
separate accepted case entry containing only the core case identity and the
runtime-v3 projection hash.  Neither the complete source fingerprint, source
envelope, discovery-record hash, discovery hash, nor offline-registry hash may
appear in an execution context or permit.

Execution is then split into two permits.  The development permit contains
only the eight seed-1009 runtime entries and the two frozen arms.  A separate
confirmation permit does not exist unless the development evidence and the
predeclared advance decision are frozen first.  Registering all forty source
fingerprints therefore does not open the 32 confirmation tracking outcomes.
Because the generator and seeds are public, this is an ordering and provenance
guarantee rather than a secrecy guarantee.

### Stage 2: development sentinel

Only seed `1009` may be opened.  Cases are run as non-overwriting independent
shards so a failure cannot erase completed evidence.  The same frozen input
object and physical-UID delivery tensor are reused by both arms.

Before truth is scored, each completed arm must pass the following runtime
audit:

- its execution authorization equals the registered case-and-arm permit;
- every topology decision reports current-page-only inputs and no posterior,
  truth, future page, or realized delivery uniform use;
- the reference attempts `2N` directed messages at every time;
- the synchronized arm follows `[2N, N, N, N]` on every aligned B4 cycle;
- the candidate attempted mask is a subset of the reference mask;
- common attempted physical edges have identical delivery outcomes;
- positive off-diagonal fusion-weight support equals the attempted topology;
- every realized rolling-B4 attempted sensor graph is strongly connected;
- all projection certificates and route hashes self-verify; and
- no silent fallback, infeasible page, or unauthorized repair occurs.

Only after this audit passes may the runner compute tracking metrics.

### Stage 3: confirmation matrix

Confirmation remains sealed until the seed-1009 development rule is evaluated
exactly once.  If it advances, all four registered confirmation seeds are run
without method, threshold, scene, metric, or exclusion changes.  The primary
matrix contains 24 pairs: two scales, three primary styles, and four seeds.
The eight crossing pairs are reported as a separate stress matrix.

## Frozen outcome metrics

Each case stores the complete per-sensor, per-time position-only Euclidean
OSPA (position E-OSPA) components and the complete position-consensus series,
then derives these summaries:

- full-horizon mean position E-OSPA;
- focus-window mean position E-OSPA using the inclusive registered scene
  window;
- worst-sensor full-horizon mean position E-OSPA;
- mean absolute cardinality error;
- mean and terminal inter-sensor OSPA disagreement;
- attempted and delivered message counts;
- attempted and delivered posterior bytes; and
- repair-page counts and the first repair time.

Relative tracking change is defined as

`(candidate - reference) / reference` when the reference is positive.  If
both arm values are zero the pair passes that metric; if the reference is
zero and the candidate is positive the pair fails and reports the absolute
change rather than manufacturing a percentage.

Communication saving is defined as

`(reference - candidate) / reference`.

Every registered communication-saving denominator must be positive.  A case
with zero reference messages or bytes does not provide a relative saving,
even if the candidate is also zero, and therefore fails closed rather than
being assigned a favorable zero change.

Both signed changes and raw arm values are retained.  Missing, non-finite, or
negative raw metric values fail closed.  No case may be removed because its
reference score is poor, its candidate score is unfavorable, or its
projection repairs more pages than expected.

`E` here means Euclidean, not extended.  The repository's legacy `ospa.m`
uses the unscaled norm of the complete four-dimensional position--velocity
state.  That mixes metres and metres per second and is therefore not used by
this gate.  The frozen V46 helper first selects state coordinates `[1, 2]`,
then computes total, localization, and cardinality OSPA with the registered
cutoff in metres and order two.  Inter-sensor OSPA applies the same projection
to every unordered sensor pair.  Velocity is excluded from both primary
tracking and consensus gates; the legacy full-state value may not rescue or
invalidate a pair.  This follows the numerical evaluation in the original
[OSPA paper](https://stochastik.math.uni-goettingen.de/preprints/ospa.pdf),
which used four-dimensional position--velocity states but retained only their
positional coordinates when computing the metric.

The evaluation object is reconstructed separately after the filter runtime
audit and contains only the frozen position-OSPA parameters and focus window
needed for scoring.  It is never passed into the filter or topology policy.
The sanitized runtime model deliberately omits OSPA parameters, ground truth,
and every derived metric; authorized state estimates and truth are joined only
inside the post-filter scorer.

Communication uses the raw sender-by-receiver-by-time ledgers emitted by the
filter.  Every attempted edge is costed after the sender's local update but
before the delivery draw and network fusion, using event type 2 and the frozen
`estimateLmbPayloadSize.m` implementation.  Its byte value is an analytical
eight-byte-per-scalar equivalent, not a capture of serialized packets; padding
and link/network/transport headers are outside the claim.  Attempted bytes
include failed deliveries and form the primary communication gate; delivered
bytes include only successful deliveries and are descriptive.  A case total
is summed over every directed non-self edge and all 160 times before the arm
ratio is formed.  Averaging per-time or per-edge savings is forbidden because
it gives a different answer when posterior sizes vary.  The raw attempted and
delivered masks, event types, scalar/byte ledgers, message counts, repair flags,
and first repair time are retained.

## Development advance rule

The seed-1009 sentinel advances only if all eight pairs complete and all runtime
audits pass.  For the six primary pairs, the following fixed thresholds apply:

- exact attempted-message saving: 37.5%;
- attempted-byte saving: at least 20% in every pair and at least 30% median;
- full-horizon mean position E-OSPA increase: at most 5% in every pair and at
  most 2% median;
- focus-window mean position E-OSPA increase: at most 5% in every pair;
- worst-sensor position E-OSPA increase: at most 10% in every pair; and
- mean position-consensus OSPA increase: at most 10% in every pair.

These are fixed-matrix development non-inferiority margins, not statistical
validation.  Crossing must complete with the same runtime contracts, but it
is summarized separately and is not allowed to change the primary advance
decision.

If the sentinel fails, confirmation stays sealed.  Any redesigned method is a
new version and requires new development and confirmation seeds.

## Confirmation interpretation

The 24 primary pairs are not 24 independent replications: one source seed is
reused across the six M24/X36 primary scene strata.  The independent unit is
therefore the complete six-scene seed block, and confirmation has only four
such units.  Each of the four blocks must independently pass the same frozen
six-pair development rule above.  A pooled mean or median cannot rescue one
failed seed or one failed scene pair.  All eight crossing stress pairs must
also complete their runtime audits, but they remain outside the primary
decision.

Four independent units are too few for an honest 5% distribution-free
significance claim: an exact one-sided sign test has only 16 sign outcomes and
a minimum attainable p-value of 1/16.  Confirmation therefore makes no
p-value, confidence-interval, or population-generalization claim.  It reports
all four raw seed-block values, their mean, median, observed range, and the
range of four leave-one-seed-out means.  Scale-specific summaries first give
the three styles equal weight within each seed, then show the four seed values.
These are descriptive seed ranges, not confidence intervals.

The committed pure evaluator makes this aggregation executable.  For each
registered metric, it first computes the frozen signed change `d(s,c)` for
seed `s` and primary scene `c`, then computes

`z(s) = mean_c d(s,c)`

over the six equally weighted scenes.  The overall mean and median, observed
range `[min_s z(s), max_s z(s)]`, and four leave-one-seed-out means are derived
only from these four `z(s)` values.  M24 and X36 summaries use the same rule
over their respective three styles and are computed independently, so an
undefined M24 relative value cannot erase an otherwise reportable X36
description.  For a tracking metric, a zero reference with a zero candidate
has change zero; a zero reference with a positive candidate fails the primary
block, marks its relative aggregate undefined, and reports the signed native-
unit difference `candidate - reference`.  It may not be dropped.  Stress-pair
numerical values remain descriptive even when such a relative change is
undefined; only their runtime-audit completion is a primary prerequisite.

The evaluator rebuilds the official registry and requires exact canonical
identity, so a caller cannot weaken thresholds and merely recompute a
self-consistent hash.  It also rejects missing, duplicated, non-finite,
arm-swapped, or schema-drifted records.  Its pass boolean is not itself a
confirmation permit: a development decision must still be frozen
independently before confirmation can open.

Communication saving may be called stable on the registered fixed matrix only
when every raw primary pair satisfies the 20% attempted-byte floor, every
seed block satisfies the 30% median floor, and all four seed-level values at
both M24 and X36 are positive.  Tracking language remains non-inferiority, not
accuracy improvement or statistical validation, and must be qualified as
observed margin compliance on the preregistered fixed matrix.

The preflight matrix, smoke, source sealer, development sentinel, and
confirmation runner are evidence stages with different scopes.  None may be
substituted for another, and structural success alone never authorizes a
tracking claim.

## Scope of the method

The runtime uses the repository's componentwise powered-Gaussian-mixture KLA
approximation.  It preserves multiple spatial modes but is not an exact
closed-form power of an arbitrary Gaussian mixture.  V46 comparisons are
therefore valid as paired experiments under this frozen receiver, while the
paper must retain the numerical-approximation qualification and must not call
the implementation exact full-density LMB-KLA.

A learned edge scorer is deferred.  It may later rank candidates inside the
first feasible minimal-edit layer, but connectivity, physical support,
distinct-receiver matching, directed `2N` execution, and rolling-window safety
remain exact projection constraints.
