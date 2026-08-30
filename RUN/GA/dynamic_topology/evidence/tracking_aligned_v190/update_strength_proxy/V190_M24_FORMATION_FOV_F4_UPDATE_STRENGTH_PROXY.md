# V190 formation-4 update-strength proxy

- Scene / seed / time: `m24-formation-fov` / `211` / `104`
- Prediction horizons: `[0 1 2]`
- Residual KLA source weight: `0.500`
- Base task risk: `0.950885`

| Source | Label | Min causal risk | Precision gain | Hard task gain | KLA task gain | Hard position gain | KLA position gain | Bytes |
|--:|:--|--:|--:|--:|--:|--:|--:|--:|
| 10 | `[25,15]` | 0.00011 | 0.00050 | +5.200% | -7.290% | +0.987% | +0.961% | 20608 |
| 8 | `[1,4]` | 0.00002 | -0.00006 | +4.110% | -8.818% | +0.781% | +0.598% | 20608 |
| 4 | `[25,15]` | 0.00025 | 0.00057 | +0.864% | +0.489% | +0.146% | +0.082% | 16576 |
| 3 | `[25,15]` | 0.00018 | 0.00046 | +0.610% | +0.339% | +0.101% | +0.056% | 16576 |
| 3 | `[17,9]` | 0.00032 | 0.00046 | +0.036% | +0.026% | +0.002% | +0.001% | 20608 |
| 5 | `[9,6]` | 0.00123 | -0.00007 | -0.546% | -10.297% | -0.339% | -0.139% | 20608 |
| 3 | `[9,6]` | 0.00149 | 0.00024 | -0.619% | -10.574% | -0.354% | -0.156% | 20608 |
| 7 | `[17,9]` | 0.00053 | 0.00072 | -4.809% | -12.958% | -0.949% | -0.354% | 20608 |
| 8 | `[17,9]` | 0.00053 | 0.00072 | -5.615% | -12.958% | -1.096% | -0.408% | 20608 |

## Evidence boundary

Truth scores a no-future-measurement posterior-risk proxy only. The causal shortlist and payload resolver remain truth-free, but this ranking is teacher triage rather than deployable policy or recursive tracking evidence.
