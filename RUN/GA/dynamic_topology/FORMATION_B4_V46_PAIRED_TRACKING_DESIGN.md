# V46 paired tracking: outcome-opening design

## Claim to test

The primary empirical claim is a communication--estimation trade-off, not an
accuracy-improvement claim.  Relative to the repaired V46 full reference, the
synchronized B4 arm should:

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
fixed now as `[1013, 1019, 1021, 1031]`.  These values do not occur in an
existing registered protocol or evidence-shard filename in this repository.
Seed choice may not be changed after any V46 tracking metric is opened.

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

Each case stores the complete per-sensor, per-time Euclidean OSPA (E-OSPA)
array and the complete consensus series, then derives these summaries:

- full-horizon mean E-OSPA;
- focus-window mean E-OSPA using the registered scene window;
- worst-sensor full-horizon mean E-OSPA;
- mean absolute cardinality error;
- mean and terminal inter-sensor OSPA disagreement;
- attempted and delivered message counts;
- attempted and delivered posterior bytes; and
- repair-page counts and the first repair time.

Relative tracking change is defined as

`(candidate - reference) / max(reference, eps)`.

Communication saving is defined as

`(reference - candidate) / reference`.

Both signed changes and raw arm values are retained.  No case may be removed
because its reference score is poor, its candidate score is unfavorable, or
its projection repairs more pages than expected.

`E` here means Euclidean, not extended.  The implementation calls `ospa.m`
with the registered Euclidean cutoff and order and retains the first (total)
OSPA component.  The evaluation object is reconstructed separately after the
filter runtime audit and contains only the frozen OSPA parameters and focus
window needed for scoring.  It is never passed into the filter or topology
policy.  The sanitized runtime model deliberately omits OSPA parameters,
ground truth, and every derived metric; authorized state estimates and truth
are joined only inside the post-filter scorer.

## Development advance rule

The seed-1009 sentinel advances only if all eight pairs complete and all runtime
audits pass.  For the six primary pairs, the following fixed thresholds apply:

- exact attempted-message saving: 37.5%;
- attempted-byte saving: at least 20% in every pair and at least 30% median;
- full-horizon mean E-OSPA increase: at most 5% in every pair and at
  most 2% median;
- focus-window mean E-OSPA increase: at most 5% in every pair;
- worst-sensor E-OSPA increase: at most 10% in every pair; and
- mean consensus-OSPA increase: at most 10% in every pair.

These are development non-inferiority margins, not statistical validation.
Crossing must complete with the same runtime contracts, but it is summarized
separately and is not allowed to change the primary advance decision.

If the sentinel fails, confirmation stays sealed.  Any redesigned method is a
new version and requires new development and confirmation seeds.

## Confirmation interpretation

The confirmation claim requires all 24 primary pairs to pass their runtime
contracts and exact count-saving rule.  Estimation non-inferiority is tested
from paired case differences, preserving scale, style, and seed strata.  The
paper must report raw paired points and uncertainty intervals in addition to
aggregate means.  A communication gain is called stable only if its lower
uncertainty bound remains positive at both scales and no scale/style stratum
depends on deleting an unfavorable seed.

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
