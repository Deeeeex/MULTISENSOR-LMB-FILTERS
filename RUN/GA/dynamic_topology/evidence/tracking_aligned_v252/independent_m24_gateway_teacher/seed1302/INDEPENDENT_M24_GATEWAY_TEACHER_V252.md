# V252 independent M24 gateway teacher dataset

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1302`
- Source commit: `e5cb7d2ddee17b568ad7e2a68004ff7492bfb4b2`
- Joint-positive anchors: `3 / 6`
- Oracle / ridge / GNN authorized: `1 / 1 / 0`
- Next decision: `include-seed-in-frozen-cross-seed-ridge-study`

| Anchor | Best available | Selected | Type | E gain | RMSE gain | Consistency gain | Byte saving | Weakest formation E / RMSE | Applied pages |
|--:|--:|--:|:--|--:|--:|--:|--:|:--|:--:|
| 40 | 6 | 1 | receiver-local-assignment | +0.000% | +0.055% | +0.000% | +0.127% | -0.000% / -0.005% | 3/3 |
| 60 | 3 | 3 | global-rank-profile | +0.047% | +5.859% | +0.321% | +0.794% | +0.000% / +0.073% | 3/3 |
| 80 | 11 | 1 | receiver-local-assignment | -0.000% | -0.017% | +0.000% | +0.306% | -0.000% / -0.144% | 3/3 |
| 100 | 10 | 10 | receiver-local-assignment | +0.160% | +0.024% | +0.512% | +0.350% | -0.000% / -0.026% | 3/3 |
| 120 | 19 | 1 | single-directed-arc | +0.001% | +0.141% | -0.000% | +0.119% | +0.000% / +0.000% | 3/3 |
| 140 | 12 | 12 | receiver-local-assignment | +0.000% | +0.194% | +0.002% | +0.306% | +0.000% / +0.000% | 3/3 |

## Aggregate selected-or-reference result

| Metric | Gain |
|:--|--:|
| E-OSPA | `+0.035%` |
| RMSE | `+1.149%` |
| Consistency | `+0.142%` |
| Attempted-byte saving | `+0.210%` |
| Weakest formation E-OSPA | `+0.000%` |
| Weakest formation RMSE | `+0.009%` |

## Frozen decision rule

At each anchor, the oracle maximizes the minimum of the E-OSPA, RMSE and inter-formation consistency percentage gains. A candidate is joint-positive only if all three gains are positive, attempted bytes do not increase and neither formation-level E-OSPA nor RMSE regresses by more than 2.0%. The requested gateway assignment persists for H=3 when physical and falls back to V242 when it is not.

## Evidence boundary

V252 uses a predeclared six-anchor time grid on three new M24 scene seeds. Seeds 1302 and 1303 are training/model-selection data; seed 1304 is opened only after the ridge representation, objective and lambda-selection rule are frozen. Seed 1305 receives no H=3 teacher labels and remains reserved for a later complete-episode test. Anchor selection uses time only, never truth, posterior risk or realized candidate outcome. Every paired arm shares measurements, delivery uniforms, filter RNG, message count and V242 continuation. These data may authorize a deployable M24 ridge policy study; they cannot establish complete-episode, X36 or paper-level gains.
