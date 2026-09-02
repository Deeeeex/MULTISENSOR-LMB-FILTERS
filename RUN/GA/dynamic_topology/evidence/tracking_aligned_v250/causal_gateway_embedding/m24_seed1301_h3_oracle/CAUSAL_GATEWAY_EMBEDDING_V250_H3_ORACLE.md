# V250 causal sensor-gateway H=3 oracle

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Source commit: `3248df0edeba8e8e24d897821a4922522cdec5a8`
- Joint-positive anchors: `3 / 3`
- Oracle / ridge / GNN authorized: `1 / 1 / 0`
- Next decision: `fit-causal-ridge-ranker-before-any-gnn`

| Anchor | Best available | Selected | Type | E gain | RMSE gain | Consistency gain | Byte saving | Weakest formation E / RMSE | Applied pages |
|--:|--:|--:|:--|--:|--:|--:|--:|:--|:--:|
| 70 | 4 | 4 | receiver-local-assignment | +0.062% | +1.051% | +0.355% | +0.043% | -0.000% / -0.027% | 3/3 |
| 84 | 17 | 17 | single-directed-arc | +0.012% | +3.099% | +1.002% | +0.667% | -0.003% / -1.153% | 3/3 |
| 151 | 13 | 13 | receiver-local-assignment | +0.219% | +0.262% | +1.440% | +0.068% | +0.000% / +0.000% | 3/3 |

## Aggregate selected-or-reference result

| Metric | Gain |
|:--|--:|
| E-OSPA | `+0.096%` |
| RMSE | `+1.316%` |
| Consistency | `+0.914%` |
| Attempted-byte saving | `+0.322%` |
| Weakest formation E-OSPA | `+0.000%` |
| Weakest formation RMSE | `+0.000%` |

## Frozen decision rule

At each anchor, the oracle maximizes the minimum of the E-OSPA, RMSE and inter-formation consistency percentage gains. A candidate is joint-positive only if all three gains are positive, attempted bytes do not increase and neither formation-level E-OSPA nor RMSE regresses by more than 2.0%. The requested gateway assignment persists for H=3 when physical and falls back to V242 when it is not.

## Evidence boundary

V250 candidate-bank preflight uses only the current physical graph, current link reliability, current positions, immutable physical identities and the causal V242 incumbent. It changes only the sensor-level embedding of cross-formation messages while preserving the V242 formation tree, local cycles, KLA weight scale and exact N+2(F-1) message count. No truth, future page, measurement, tracking outcome or posterior proxy is used. Structural diversity is not a tracking-gain or generalization claim.
