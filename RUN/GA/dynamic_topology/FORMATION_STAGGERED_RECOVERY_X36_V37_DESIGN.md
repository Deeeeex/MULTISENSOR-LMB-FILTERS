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

## Frozen truth-free controller traces

The initial source metrics and complete H=3 controller traces were frozen
before any X36 tracking outcome was scored:

| Anchor | Initial bank action | Retained formations over H=3 | Explicit release | Reference fallback | Attempted bytes |
|--:|--:|:--|:--|:--|:--|
| 72 | 53 | `[3 5 6] -> 5 -> 2` | none | `[0 0 0]` | `[3332184 3581032 3868936]` |
| 100 | 52 | `[1 2 5 6] -> [1 2 4 5] -> [4 6]` | formation 6 at t=101 | `[0 0 0]` | `[2861728 2713960 2759840]` |
| 128 | 44 | `[1 2 4 6] -> [1 3 4] -> [3 6]` | none | `[0 0 0]` | `[1949704 1884000 1954304]` |

The full 35-field runtime fingerprints are frozen as:

| Anchor | Executed action indices | Runtime fingerprint SHA-256 |
|--:|:--|:--|
| 72 | `[53 17 3]` | `fef95ee229bc6c4a86142ed72e3f6455d3dfac807a63b92ac2136bf905431f7a` |
| 100 | `[52 28 41]` | `8f71e1c1e37cfb7d63f1f481788f9fea2277f69940d0bca52f5878b7c47b7a4c` |
| 128 | `[44 14 37]` | `c39e67d450a697632a41b807272bfdd20ccb4b7695bca08e04dbb37668221067` |

All three states use the same posterior- and current-link-aware,
retention-debt receding-horizon controller. Only t=100 invokes the explicit
mature-formation release schedule. The broader X36 mechanism is therefore
debt-aware protection and rotation; staggered release is one conditional
behavior of that controller, not a step that must appear in every state.

## Authorization sequence

1. Reproduce the same-hardware scene and load gates from clean source.
2. Generate and hash the three reference posterior caches in one trajectory.
3. Construct the unchanged v35 controller at each anchor and freeze its exact
   initial source metrics and action.
4. Execute the complete H=3 controller without scoring tracking; freeze every
   suspension, release, fallback, posterior/link-use, and rolling-B3 field.
5. Reproduce those traces from a later clean commit.

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
