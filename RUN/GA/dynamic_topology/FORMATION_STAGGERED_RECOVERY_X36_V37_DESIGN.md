# V37 X36 source-only scale gate

## Why this X36 scene is retained

The same-hardware `x36-formation-fov` scene keeps the M24 sensor contract:
120-degree total FoV, 300 m hard range, shared formation boresight, detection
probability, clutter, noise, and per-sensor quality scaling. On seed 211, the
focus-window visible-target load is 13.3750 for M24 and 13.3291 for X36, a
relative difference of about -0.34%. Both have zero focus-window blackout.

X36 nevertheless increases formations from four to six, sensors from 24 to
36, targets from 16 to 24, focus handovers from 31 to 68, and mean physical
inter-formation edges from 216 to about 493. It therefore tests scale and route
combinatorics without silently replacing the sensor hardware or making nearly
all targets visible.

## Frozen source states

V37 uses X36 seed 211 only and opens one anchor inside each registered paired
blockage window:

| Anchor | Registered blockage pair | Return window |
|--:|:--|:--|
| 72 | formations 1 and 2 | 72--74 |
| 100 | formations 3 and 4 | 100--102 |
| 128 | formations 5 and 6 | 128--130 |

The anchors are fixed before generating any X36 posterior. A single fixed-CCW
reference trajectory produces all three predecision caches. Cache generation,
scene auditing, initial v35 control construction, and later full H=3 source
replay use no tracking truth or future outcome.

## Revoked v2 traces and the v3 causal boundary

The revoked v2 run recorded the following candidate-side source metrics before
any X36 tracking outcome was scored. The values remain preregistered
development expectations for drift detection, but they are not accepted as
causal evidence because the v2 callback could reach forbidden inputs:

| Anchor | Initial bank action | Retained formations over H=3 | Explicit release | Reference fallback | Attempted bytes |
|--:|--:|:--|:--|:--|:--|
| 72 | 53 | `[3 5 6] -> 5 -> 2` | none | `[0 0 0]` | `[3332184 3581032 3868936]` |
| 100 | 52 | `[1 2 5 6] -> [1 2 4 5] -> [4 6]` | formation 6 at t=101 | `[0 0 0]` | `[2861728 2713960 2759840]` |
| 128 | 44 | `[1 2 4 6] -> [1 3 4] -> [3 6]` | none | `[0 0 0]` | `[1949704 1884000 1954304]` |

The old candidate-only fingerprints are retained only to identify the revoked
artifact:

| Anchor | Executed action indices | Revoked v2 candidate SHA-256 |
|--:|:--|:--|
| 72 | `[53 17 3]` | `fef95ee229bc6c4a86142ed72e3f6455d3dfac807a63b92ac2136bf905431f7a` |
| 100 | `[52 28 41]` | `8f71e1c1e37cfb7d63f1f481788f9fea2277f69940d0bca52f5878b7c47b7a4c` |
| 128 | `[44 14 37]` | `c39e67d450a697632a41b807272bfdd20ccb4b7695bca08e04dbb37668221067` |

V3 replaces the self-reported truth-free claim with a structural callback
boundary. The policy receives only the current posterior, the current 2-D
drop-probability page, current node positions and physical action set, past
selected-topology and update histories, static fusion parameters, and sensor
formation IDs. The posterior itself is projected to a registered LMB schema
(label, existence, GM state, covariance, and association summaries); trajectory
history, timestamps, unknown side-channel fields, and function handles cannot
reach the callback. Its model view contains only the state dimension, existence
threshold, an empty object template, and formation IDs. Target trajectories,
the complete dynamic scenario, measurements, sensor trajectories, link
uniforms, future drop-probability pages, callback handles, and future-dated
posterior timestamps are rejected before the callback executes. Every other
callback field is restricted to primitive numeric/logical/character or
cell/struct containers, with exact adjacency, edge-score, budget, and history
shapes tied to the current time; arbitrary class objects cannot carry a hidden
oracle into the policy.

Each anchor must now reproduce two separately identified traces:

| Anchor | Reference arm | Candidate arm | Reference SHA-256 | Candidate SHA-256 |
|--:|:--|:--|:--|:--|
| 72 | fixed CCW, posterior/link adaptation forbidden | causal v35, posterior/link use required | `2d3ff282f22946283e7a69fbaebbdfe68ddce2b3761e7bccd999d0ca2fcd15cc` | `9a9930347c3c5f176c4bc0b3fd7529d8a6ffd05fdbc6c1cb3516c4fd49204eab` |
| 100 | fixed CCW, posterior/link adaptation forbidden | causal v35, posterior/link use required | `712198520a745ca98089a053b2a3391b3280bf35c173f39344af98f3972a3425` | `c296d38b885b9e66ab0be2fabf9696f87e908bc38b70af92619cdaf1a8f213a1` |
| 128 | fixed CCW, posterior/link adaptation forbidden | causal v35, posterior/link use required | `afff6228ef68c3d3d7c7f51c18e67884807e448f7edf126d5a670dd3bb570f81` | `6ff4e90f72563d410c7f3bfe60363c08d6d34ccd919dbf242966664f31b02881` |

The six fingerprints above were discovered from clean commit `58492d9` and
published as an atomic MAT/report/completion-marker set. The discovery artifact
remained outcome-unauthorized: all three eligibility flags were false, all
tracking/truth/future-use flags were false, and the reference/candidate input-use
attestations were respectively `posterior=0, link=0` and
`posterior=1, link=1`. The next clean commit must reproduce the full matrix
before any outcome permit can be minted.

Only t=100 invokes the explicit mature-formation release schedule. The broader
candidate mechanism is debt-aware protection and rotation; staggered release
is one conditional behavior, not a step that must appear in every state.

## Authorization sequence

1. Reproduce the same-hardware scene and load gates from clean source.
2. Generate and hash the three reference posterior caches in one trajectory.
3. Construct the unchanged v35 controller at each anchor and freeze its exact
   initial source metrics and action.
4. Execute both the fixed reference and causal candidate through the complete
   H=3 source-only runtime without scoring tracking.
5. Freeze all six fingerprints, including arm identity, input-contract
   attestation, suspension/release state, fallback, posterior/link-use, fusion
   weights, attempted bytes, and rolling-B3 fields.
6. Reproduce all six traces from a later clean commit before minting a new
   immutable outcome permit.

The source gate publishes a MAT file, a readable report, and a completion
marker. The marker is written last and binds the SHA-256 hashes of the other
two files. A MAT/report pair without that marker is an interrupted publication,
not admissible evidence; the later loader must verify the marker and both
hashes before it can mint an outcome permit.

The same gate hashes every Git-registered repository-local `.m`, `.p`, `.mlx`,
`.oct`, and `.mex*` executable source (including symlinks) and rejects any
unregistered executable source, including ignored shadow files. It rechecks
that manifest before and after every arm, before eligibility, and immediately
before publishing the completion marker.

The clean v2 preflight from commit `57f65d0` reproduced all three states. Its
MAT SHA-256 is
`de3b17cc5f67427f5c530a6d61232c34bde467ca9d3b93ebe9dd577610180d64`.
It records `3/3` eligible pair proposals but, by construction, authorizes zero
tracking runs and a maximum authorized count of zero.

This v2 artifact is now revoked. Adversarial review found that removing the
three top-level truth fields did not remove nested target trajectories from the
model, and the runtime policy callback could still see future link uniforms and
future drop-probability pages. The controller did not declare using them, but
reachability alone invalidates the claimed causal-input proof. No X36 outcome
was retained. V3 must structurally expose only current observable inputs and
freeze both reference and candidate H=3 fingerprints before a new permit is
minted.

The v2 permit and zero-argument exact-two-arm runner are fail-closed and may
not execute the revoked preflight. A replacement runner may execute
only one reference-versus-v35 pair per state and writes to the separate
`x36_outcome/screen` root without overwrite. The aggregate outcome gate remains
unchanged from M24: at least two of three strict-strong states, median mean
tracking gain at least 2%, no state below -1%, and at least two positive
terminal-consensus states. The permit, runner, generic executor, posterior
safety bank, runtime-fingerprint builder, and observable-context boundary must
be source-hash-bound and reproduced from a clean commit before the exact X36
outcome is opened.
GNN training, X48, reserved seeds, and validation claims remain unauthorized.
