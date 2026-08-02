# V34 gateway label-pruning threshold preflight

- Contract / generation commit: `formation-gateway-pruning-threshold-v34-preflight-v1 / 3df4b4a053d14656e36baa3d6953af1cfe1ca93c`
- Tracked dirty / untracked source: `0 / 0`
- Preset / seed / reconnect time: `m24-formation-fov / 211 / 74`
- Source cache SHA-256: `60dfbf2615181cde046af15f42bba37c415ea0034cb7ce53685b79042bfaf762`
- Source candidates / low grid: `[8 10 12] / [0.0001 0.00025 0.0005 0.001 0.0025 0.005 0.01]`
- Safe / useful / eligible pairs: `0 / 0 / 0`
- Maximum safe weights: `[NaN NaN NaN]`
- Minimum useful low-grid weights: `[NaN NaN NaN]`
- Selected action / weight: `reference / NaN`
- Tracking scored / authorized: `0 / 0`
- Replay / control construction: `137.34 / 128.96 s`

## Low-weight grid

| Candidate | Weight | Disagreement improvement | Retention risk | Min label retention | Threshold drops | Safe | Useful | Eligible |
|--:|--:|--:|--:|--:|--:|:--:|:--:|:--:|
| 8 | 0.00010 | -2.8721% | 0.009567 | 0.027550 | 0 | 0 | 0 | 0 |
| 8 | 0.00025 | -2.8675% | 0.010506 | 0.027550 | 0 | 0 | 0 | 0 |
| 8 | 0.00050 | -2.8607% | 0.011105 | 0.026950 | 0 | 0 | 0 | 0 |
| 8 | 0.00100 | -2.8448% | 0.011585 | 0.026950 | 0 | 0 | 0 | 0 |
| 8 | 0.00250 | -2.7876% | 0.013408 | 0.026950 | 0 | 0 | 0 | 0 |
| 8 | 0.00500 | -2.6745% | 0.017195 | 0.026950 | 0 | 0 | 0 | 0 |
| 8 | 0.01000 | -2.2779% | 0.032006 | 0.026950 | 0 | 0 | 0 | 0 |
| 10 | 0.00010 | -2.9371% | 0.008805 | 0.027956 | 0 | 0 | 0 | 0 |
| 10 | 0.00025 | -2.9338% | 0.009693 | 0.027956 | 0 | 0 | 0 | 0 |
| 10 | 0.00050 | -2.9293% | 0.010205 | 0.027348 | 0 | 0 | 0 | 0 |
| 10 | 0.00100 | -2.9184% | 0.010490 | 0.027348 | 0 | 0 | 0 | 0 |
| 10 | 0.00250 | -2.8757% | 0.011699 | 0.027348 | 0 | 0 | 0 | 0 |
| 10 | 0.00500 | -2.7688% | 0.015078 | 0.027348 | 0 | 0 | 0 | 0 |
| 10 | 0.01000 | -2.3679% | 0.028982 | 0.027348 | 0 | 0 | 0 | 0 |
| 12 | 0.00010 | -2.8835% | 0.008980 | 0.026274 | 0 | 0 | 0 | 0 |
| 12 | 0.00025 | -2.8796% | 0.009881 | 0.026274 | 0 | 0 | 0 | 0 |
| 12 | 0.00050 | -2.8741% | 0.010415 | 0.025702 | 0 | 0 | 0 | 0 |
| 12 | 0.00100 | -2.8612% | 0.010747 | 0.025702 | 0 | 0 | 0 | 0 |
| 12 | 0.00250 | -2.8163% | 0.012000 | 0.025702 | 0 | 0 | 0 | 0 |
| 12 | 0.00500 | -2.7209% | 0.014861 | 0.025702 | 0 | 0 | 0 | 0 |
| 12 | 0.01000 | -2.3373% | 0.028173 | 0.025702 | 0 | 0 | 0 | 0 |

## Decision

No low-weight alternative gateway is both safe and useful on the frozen grid. The report brackets label survival below the v33 useful-weight region and stops before tracking.

## Evidence boundary

v34 is a source-only threshold diagnostic on the same frozen v30 t=74 state and v32 routes 8, 10, and 12. It evaluates only the preregistered low alternative-edge weights from 0.0001 through 0.010, below the v33 grid. The topology, payload count, dominant route, and formation graph remain fixed. Exact label retention and one-round disagreement may identify a safe-useful pair, but no truth or future outcome is read. Tracking is authorized only after a clean frozen preflight finds a pair passing all safety and 0.25-percent improvement gates. Otherwise the observed safe and useful weight intervals are reported without opening another M24 state, GNN, X36, X48, or validation.
