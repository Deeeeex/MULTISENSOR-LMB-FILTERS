# V240 causal minimal-edit formation-tree preflight

- Seed: `41`
- Structural gate passed: `1`
- Tracking outcome authorized: `0`

| Preset | Sensors | Formations | Targets | Scene | Physical connected | Initial tree fails | First failure | Tree changes | Reselection times | Replacements | Messages/round | Strong | Physical route | Min sensor-target (m) | Blackout | Focus blackout | Worst target | Longest | Visible sensors | Expected detections | Handovers |
|:--|--:|--:|--:|:--:|:--:|:--:|--:|--:|:--|--:|--:|:--:|:--:|--:|--:|--:|--:|--:|--:|--:|--:|
| m24-formation-fov-formation-braid | 24 | 4 | 16 | 1 | 1 | 1 | 71 | 2 | [71 149] | 2 | 48 | 1 | 1 | 43.3 | 0.042 | 0.067 | 0.231 | 37 | 5.16 | 3.60 | 22 |
| x36-formation-fov-formation-braid | 36 | 6 | 24 | 1 | 1 | 1 | 55 | 4 | [55 95 108 153] | 4 | 72 | 1 | 1 | 42.8 | 0.028 | 0.045 | 0.225 | 18 | 5.08 | 3.59 | 40 |
| x48-formation-fov-formation-braid | 48 | 8 | 32 | 1 | 1 | 1 | 45 | 6 | [45 71 102 103 135 149] | 6 | 96 | 1 | 1 | 43.2 | 0.045 | 0.043 | 0.281 | 43 | 5.07 | 3.54 | 51 |

## Decision boundary

V240 is a causal development policy for the exploratory formation-braid scenes. It preserves the preceding formation tree whenever that tree remains physically feasible. On physical infeasibility only, it selects a current spanning tree lexicographically by minimum formation-edge replacement, maximum bottleneck reliability, maximum total log reliability and minimum distance. It reads no truth, future outcome or posterior payload. Every receiver retains the V227 two-input KLA weight contract. Structural results do not establish tracking gain or generalization.
