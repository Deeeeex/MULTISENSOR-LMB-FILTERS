# V255 budgeted local gateway action structure

- Analysis source commit: `01c03e692d2cad53e1a4de9c986019b5038db117`
- V252 source commits: `e5cb7d2ddee17b568ad7e2a68004ff7492bfb4b2`
- Development windows: `18`
- Maximum changed directed gateway arcs: `1`
- Incremental development byte cap relative to V242: `2.0%`
- Exact-net positive window coverage: `10/18`
- Credit-positive window coverage: `13/18`
- Unrestricted credit-positive coverage: `13/18`
- Coverage preserved by one-arc restriction: `1`
- Positive joint-score sum retained: `0.787`
- Next decision: `freeze-v255-single-arc-multi-output-model-before-seed1306`

| Seed | Anchor | Local control B | Exact-net positive | Credit positive local / all | Best joint local / all | Retention |
|--:|--:|--:|--:|:--|:--|--:|
| 1302 | 40 | 408 | 0 | 3 / 3 | +0.0002% / +0.0002% | 1.000 |
| 1302 | 60 | 408 | 1 | 2 / 4 | +0.0380% / +0.0475% | 0.800 |
| 1302 | 80 | 408 | 0 | 0 / 0 | +0.0000% / +0.0000% | 0.000 |
| 1302 | 100 | 408 | 2 | 4 / 5 | +0.0242% / +0.0942% | 0.257 |
| 1302 | 120 | 408 | 0 | 0 / 0 | +0.0000% / +0.0000% | 0.000 |
| 1302 | 140 | 408 | 3 | 5 / 5 | +0.0023% / +0.0023% | 1.000 |
| 1303 | 40 | 408 | 0 | 0 / 0 | +0.0000% / +0.0000% | 0.000 |
| 1303 | 60 | 408 | 0 | 0 / 0 | +0.0000% / +0.0000% | 0.000 |
| 1303 | 80 | 408 | 1 | 2 / 3 | +0.0146% / +0.0146% | 1.000 |
| 1303 | 100 | 408 | 1 | 1 / 1 | +0.0008% / +0.0008% | 1.000 |
| 1303 | 120 | 408 | 0 | 3 / 3 | +0.0074% / +0.0074% | 1.000 |
| 1303 | 140 | 408 | 2 | 4 / 4 | +0.2845% / +0.2845% | 1.000 |
| 1304 | 40 | 408 | 0 | 0 / 0 | +0.0000% / +0.0000% | 0.000 |
| 1304 | 60 | 408 | 1 | 3 / 3 | +0.0010% / +0.0010% | 1.000 |
| 1304 | 80 | 408 | 0 | 2 / 4 | +0.0416% / +0.1540% | 0.270 |
| 1304 | 100 | 408 | 2 | 7 / 8 | +0.0911% / +0.0911% | 1.000 |
| 1304 | 120 | 408 | 2 | 3 / 3 | +0.2029% / +0.2029% | 1.000 |
| 1304 | 140 | 408 | 1 | 2 / 2 | +0.0009% / +0.0009% | 1.000 |

## Interpretation

A one-directed-arc action preserves every development window that has a positive action under the registered local communication-credit screen. It also leaves positive exact-net actions in at least half of the windows. Global additive composition therefore adds unsupported extrapolation without increasing positive window coverage in these data. This authorizes only the V255 multi-output ridge screen and the new seed-1306 teacher; it is not a deployable tracking result.

## Evidence boundary

V255 is a development-stage repair of the V242 minimum causal backbone. One action may replace at most one directed physical gateway while preserving the formation tree, local cycles, KLA weights and N+2(F-1) posterior-message count. Controller telemetry is collected once per H=3 hold and only from the source and receiver formations. The learned model predicts the six outcome coordinates separately; a scalar minimum-slack regression is not reused. Seeds 1302--1304 are development data, seed 1306 is a new frozen holdout, and seed 1305 remains untouched for a later complete episode. Topology and communication-credit constraints are deterministic; tracking gains require paired evidence and do not authorize a GNN, X36 or a paper claim.
