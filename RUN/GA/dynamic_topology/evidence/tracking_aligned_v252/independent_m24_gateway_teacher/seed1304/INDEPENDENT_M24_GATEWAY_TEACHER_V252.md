# V252 independent M24 gateway teacher dataset

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1304`
- Source commit: `e5cb7d2ddee17b568ad7e2a68004ff7492bfb4b2`
- Joint-positive anchors: `4 / 6`
- Oracle / ridge / GNN authorized: `1 / 1 / 0`
- Next decision: `include-seed-in-frozen-cross-seed-ridge-study`

| Anchor | Best available | Selected | Type | E gain | RMSE gain | Consistency gain | Byte saving | Weakest formation E / RMSE | Applied pages |
|--:|--:|--:|:--|--:|--:|--:|--:|:--|:--:|
| 40 | 9 | 1 | receiver-local-assignment | +0.074% | +0.007% | +0.000% | +0.637% | -0.000% / -0.151% | 3/3 |
| 60 | 19 | 19 | single-directed-arc | +0.065% | +0.044% | +0.001% | +0.331% | +0.000% / +0.000% | 3/3 |
| 80 | 21 | 1 | single-directed-arc | +0.085% | +1.523% | +0.000% | +0.671% | +0.000% / +0.000% | 3/3 |
| 100 | 8 | 8 | receiver-local-assignment | +0.091% | +13.609% | +0.507% | +0.000% | +0.000% / +0.000% | 3/3 |
| 120 | 18 | 18 | single-directed-arc | +0.203% | +11.921% | +0.387% | +0.272% | -0.333% / +0.000% | 3/3 |
| 140 | 11 | 11 | receiver-local-assignment | +0.001% | +0.106% | +0.003% | +2.839% | +0.000% / +0.000% | 3/3 |

## Aggregate selected-or-reference result

| Metric | Gain |
|:--|--:|
| E-OSPA | `+0.062%` |
| RMSE | `+7.249%` |
| Consistency | `+0.158%` |
| Attempted-byte saving | `+0.535%` |
| Weakest formation E-OSPA | `-0.041%` |
| Weakest formation RMSE | `+0.076%` |

## Frozen decision rule

At each anchor, the oracle maximizes the minimum of the E-OSPA, RMSE and inter-formation consistency percentage gains. A candidate is joint-positive only if all three gains are positive, attempted bytes do not increase and neither formation-level E-OSPA nor RMSE regresses by more than 2.0%. The requested gateway assignment persists for H=3 when physical and falls back to V242 when it is not.

## Evidence boundary

V252 uses a predeclared six-anchor time grid on three new M24 scene seeds. Seeds 1302 and 1303 are training/model-selection data; seed 1304 is opened only after the ridge representation, objective and lambda-selection rule are frozen. Seed 1305 receives no H=3 teacher labels and remains reserved for a later complete-episode test. Anchor selection uses time only, never truth, posterior risk or realized candidate outcome. Every paired arm shares measurements, delivery uniforms, filter RNG, message count and V242 continuation. These data may authorize a deployable M24 ridge policy study; they cannot establish complete-episode, X36 or paper-level gains.
