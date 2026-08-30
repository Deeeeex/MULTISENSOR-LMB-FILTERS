# V190 formation-5 update-strength proxy

- Scene / seed / time: `x36-formation-fov` / `211` / `72`
- Prediction horizons: `[0 1 2]`
- Residual KLA source weight: `0.500`
- Base task risk: `0.668243`

| Source | Label | Min causal risk | Precision gain | Hard task gain | KLA task gain | Hard position gain | KLA position gain | Bytes |
|--:|:--|--:|--:|--:|--:|--:|--:|--:|
| 2 | `[13,11]` | 0.00677 | -0.00004 | +2.478% | -9.832% | +0.717% | +0.596% | 20608 |
| 20 | `[7,5]` | 0.00051 | 0.00114 | +0.557% | +0.395% | +0.252% | +0.195% | 20608 |
| 32 | `[25,18]` | 0.00049 | 0.00134 | +0.151% | +0.146% | +0.028% | +0.020% | 20608 |
| 36 | `[31,21]` | 0.00232 | 0.00247 | -0.062% | -9.830% | -0.112% | -0.041% | 20608 |
| 34 | `[31,21]` | 0.00148 | 0.00245 | -0.168% | -11.936% | -0.156% | -0.063% | 20608 |
| 4 | `[31,22]` | 0.00448 | 0.00185 | -0.190% | -9.183% | -0.142% | -0.076% | 16576 |
| 34 | `[1,1]` | 0.00051 | 0.00031 | -0.575% | -0.290% | -0.244% | -0.125% | 16576 |
| 19 | `[1,4]` | 0.00831 | 0.00090 | -1.430% | -12.027% | -0.586% | -0.258% | 20608 |
| 6 | `[1,4]` | 0.00794 | 0.00116 | -1.698% | -12.068% | -0.796% | -0.387% | 20608 |
| 2 | `[1,4]` | 0.00674 | 0.00113 | -2.066% | -12.068% | -0.941% | -0.467% | 20608 |
| 20 | `[7,7]` | 0.00036 | 0.00112 | -3.390% | -10.685% | -1.355% | -0.635% | 20608 |
| 19 | `[7,7]` | 0.00022 | 0.00123 | -3.502% | -10.685% | -1.412% | -0.644% | 20608 |

## Evidence boundary

Truth scores a no-future-measurement posterior-risk proxy only. The causal shortlist and payload resolver remain truth-free, but this ranking is teacher triage rather than deployable policy or recursive tracking evidence.
