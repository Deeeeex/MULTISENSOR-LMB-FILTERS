# V252 independent M24 gateway teacher dataset

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1303`
- Source commit: `e5cb7d2ddee17b568ad7e2a68004ff7492bfb4b2`
- Joint-positive anchors: `4 / 6`
- Oracle / ridge / GNN authorized: `1 / 1 / 0`
- Next decision: `include-seed-in-frozen-cross-seed-ridge-study`

| Anchor | Best available | Selected | Type | E gain | RMSE gain | Consistency gain | Byte saving | Weakest formation E / RMSE | Applied pages |
|--:|--:|--:|:--|--:|--:|--:|--:|:--|:--:|
| 40 | 10 | 1 | receiver-local-assignment | +0.000% | +0.123% | +0.000% | +0.176% | -0.000% / -0.007% | 3/3 |
| 60 | 9 | 1 | receiver-local-assignment | -0.145% | +0.381% | -0.373% | +0.769% | -0.319% / +0.000% | 3/3 |
| 80 | 7 | 7 | receiver-local-assignment | +0.005% | +0.197% | +0.000% | +0.223% | +0.000% / +0.000% | 3/3 |
| 100 | 4 | 4 | receiver-local-assignment | +0.001% | +0.143% | +0.001% | +0.231% | +0.000% / +0.000% | 3/3 |
| 120 | 16 | 16 | single-directed-arc | +0.007% | +1.807% | +0.155% | +0.051% | -0.000% / -0.000% | 3/3 |
| 140 | 5 | 5 | receiver-local-assignment | +0.284% | +5.530% | +3.684% | +0.080% | -0.000% / +0.000% | 3/3 |

## Aggregate selected-or-reference result

| Metric | Gain |
|:--|--:|
| E-OSPA | `+0.049%` |
| RMSE | `+1.587%` |
| Consistency | `+0.632%` |
| Attempted-byte saving | `+0.095%` |
| Weakest formation E-OSPA | `+0.000%` |
| Weakest formation RMSE | `+0.001%` |

## Frozen decision rule

At each anchor, the oracle maximizes the minimum of the E-OSPA, RMSE and inter-formation consistency percentage gains. A candidate is joint-positive only if all three gains are positive, attempted bytes do not increase and neither formation-level E-OSPA nor RMSE regresses by more than 2.0%. The requested gateway assignment persists for H=3 when physical and falls back to V242 when it is not.

## Evidence boundary

V252 uses a predeclared six-anchor time grid on three new M24 scene seeds. Seeds 1302 and 1303 are training/model-selection data; seed 1304 is opened only after the ridge representation, objective and lambda-selection rule are frozen. Seed 1305 receives no H=3 teacher labels and remains reserved for a later complete-episode test. Anchor selection uses time only, never truth, posterior risk or realized candidate outcome. Every paired arm shares measurements, delivery uniforms, filter RNG, message count and V242 continuation. These data may authorize a deployable M24 ridge policy study; they cannot establish complete-episode, X36 or paper-level gains.
