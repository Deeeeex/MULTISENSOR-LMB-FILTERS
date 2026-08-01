# Formation-FoV value-gated joint source-trust validation protocol

This protocol is frozen before any registered tracking outcome is opened.
Geometry-only calibration artifacts do not count as tracking outcomes.

## Frozen method and comparison

- Candidate: `adaptive-dominant-composite-balanced-certified-overlap-consensus-joint-trust-valuefloor000-reference-cap-payload-margin020-e05-a70`
- Reference: `backbone-residual-spliced-cycle-ccw-a70-e05`
- Frozen development summary SHA-256: `6892560a13d314149e7264984e3a7076e6374a9b5625eb41d26b771e3032de16`
- Every pair uses the same scenario, measurements, link uniforms, filter seed offset, continuation state and code commit for both arms.
- Partial outcomes may diagnose runtime failures only. They must not tune the candidate, windows, gates or reference.

## Registered cases

| Scene | Nodes | Formations | Sensors per formation | Window | Joint actions per step | Seeds |
|:--|--:|--:|--:|:--:|--:|:--|
| `m24-formation-fov` | 24 | 4 | 6 | 70:90 | 256 | 83, 89, 97, 101, 103 |
| `x36-formation-fov` | 36 | 6 | 6 | 60:80 | 4096 | 83, 89, 97, 101, 103 |

Both scenes use the same `formation-shared-120deg-r300-q300-v1` sensor
hardware profile: a 120-degree field of view, a hard 300 m sensing range,
and a formation-shared scene-center boresight rule. Each 21-step window is
the scene's first registered inter-formation blockage interval.

## Per-scene acceptance gates

- Mean E-OSPA improvement is at least 5%.
- All five paired seeds have positive E-OSPA improvement.
- Aggregate worst-node E-OSPA and aggregate consensus disagreement do not regress.
- Aggregate attempted-byte saving is at least 2%.
- No seed increases attempted bytes by more than 0.5%.
- Every pair passes the registered topology, payload, truth-use, fallback and exact joint-action-count safety checks.

The aggregate validation result is available only after all ten pairs are
opened and both scenes pass their gates independently.

## Interpretation boundary

This is a paired conditional-continuation test of the first blockage window.
Passing it would support a local M24/X36 formation-FoV claim. It would not
support a full-episode claim or an X48 scale-generalization claim; those
require subsequent frozen experiments.
