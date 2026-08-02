# V37 X36 same-hardware scene audit

- Contract / generation commit: `formation-staggered-recovery-x36-v37-scene-audit-v1 / a2d48c89c7d7b9b0bf2e939a77a9e1bbaa5f5d87`
- Hardware contract passed: `1`
- Scene gate passed: `1`
- Tracking scored / posterior generated: `0 / 0`

## Matched sensing load

| Scene | Sensors | Targets | Focus visible targets / sensor | Focus blackout | Multi-formation visibility | Handovers | Mean physical inter-formation edges |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `m24-formation-fov` | 24 | 16 | 13.375000 | 0.000000 | 0.780923 | 31 | 216.000 |
| `x36-formation-fov` | 36 | 24 | 13.329134 | 0.000000 | 0.786012 | 68 | 493.169 |

- X36/M24 focus-load error: `-0.3429%`
- X36 maximum per-target blackout: `19.3750%`
- X36 maximum consecutive blackout: `19` steps
- X36 ownership entropy: `0.998983`

## Decision

X36 preserves the registered 120-degree, 300 m sensor hardware and matches M24 local sensing load within 1%. Its larger number of formations, targets, handovers, and physical cross-formation choices makes it a scale/route test rather than a hidden FoV stress change. The three fixed-reference source caches may now be generated; no X36 tracking outcome is authorized.

## Evidence boundary

This audit uses scenario geometry and visibility only. It compares M24 and X36 under identical sensor hardware and checks that the per-sensor focus load remains matched without hiding blackout or handover failures. It generates no posterior and scores no tracking outcome. Passing authorizes only the registered X36 fixed-reference cache trajectory.
