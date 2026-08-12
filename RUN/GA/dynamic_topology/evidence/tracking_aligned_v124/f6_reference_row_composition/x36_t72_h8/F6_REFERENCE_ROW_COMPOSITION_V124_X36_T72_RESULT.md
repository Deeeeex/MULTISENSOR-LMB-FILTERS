# V124 F6-reference row composition: X36 t72 H=8

- V113 baselines reused: `1`
- V123 candidate screen reused: `0`
- Gate passed: `0`

| Mean E-OSPA | Gain vs CW | vs V113 | Mature min | Min form. | Terminal form. | F6 peers | Worst | Bytes | Gate |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| 78.553292 | +3.973% | -0.094% | +2.026% | -1.495% | -5.865% | -7.206% | +6.850% | +2.650% | 0 |

## Diagnostics

- Formation gains: `[0.3078 0.9785 10.62 3.754 9.913 -1.495]%`
- Terminal formation gains: `[4.351 4.858 16.38 9.566 6.645 -5.865]%`
- Per-page gains: `[0.9093 3.021 4.154 2.773 2.026 7.242 5.748 6.28]%`
- Window / terminal consensus: `+10.829% / +15.846%`
- Rolling B3: `1`

## Evidence boundary

V124 is a privileged opened-development X36 seed-211 t=72 H=8 single-ablation of V123. It keeps every V123 carrier-row and payload-participation choice except that F6 uses the clockwise reference row on all pages, removing the opened delayed reversal from the F1-to-F6 row. The resulting sequence needs no additional connectivity fallback and preserves 60 messages, physicality, the weight multiset and rolling B3. Measurements, delivery uniforms, filter RNG and communication accounting remain paired. This is an action-space upper bound, not a deployable policy, validation or generalization evidence.
