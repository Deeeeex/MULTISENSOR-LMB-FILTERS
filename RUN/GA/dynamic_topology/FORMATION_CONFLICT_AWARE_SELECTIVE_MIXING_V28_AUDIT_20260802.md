# V28 conflict-aware selective-mixing audit

## Decision

**The v28 primary gate fails with `0/4` strong candidates.** Do not open another M24 state, train a GNN, or run X36/X48 under this protocol. The mechanism direction is retained, but persistent single-edge damping is too weak to meet the paper-level target.

- Contract: `formation-conflict-aware-selective-mixing-v28-audit-v1`
- Source generation commit: `bf641fec9f34f8a432daab4b91e259701fd2c7c2`
- Source cache SHA-256: `60dfbf2615181cde046af15f42bba37c415ea0034cb7ce53685b79042bfaf762`
- Preset / seed / time: `m24-formation-fov / 211 / 72`
- Return times: `[72 73 74]`
- New tracking state opened by audit: `0`

| Action | Predicted one-step disagreement change | Mean gain | Worst gain | Min formation | Window consensus | Final consensus | Mean card. error | Byte saving | Strong |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference` | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 2.708333 | +0.000% | 0 |
| `damp-reference-f1` | +0.111% | -0.000% | +0.000% | -0.000% | -0.000% | -0.002% | 2.708333 | +0.000% | 0 |
| `damp-reference-f2` | +0.990% | +0.856% | +0.101% | +0.000% | +2.906% | +1.289% | 2.638889 | -0.339% | 0 |
| `damp-reference-f3` | +0.399% | +0.721% | +0.000% | +0.000% | +2.949% | +2.818% | 2.666667 | -0.324% | 0 |
| `damp-reference-f4` | +0.395% | +0.683% | +0.000% | +0.000% | +2.317% | +3.832% | 2.652778 | +0.000% | 0 |

## What remains valid

Three of four nonreference actions improve mean tracking, window consensus, terminal consensus, and mean cardinality error in the same direction. The best mean gain is `0.856%`; the best window-consensus gain is `2.949%`. This supports selective conflict attenuation as a mechanism, but the magnitude is below the frozen 2% tracking threshold.

## Why the next action must change

The one-step disagreement guard predicts a risk increase for every damping action, while three actions improve the realized H=3 consensus. Across the four nonreference actions, predicted one-step risk change versus realized window-consensus gain has Pearson `r = 0.710610`. This repeats the earlier warning: a current one-round summary metric is not a recursive consensus certificate.

V29 should therefore test **one-step temporal suspension** of reference cross-formation edges rather than persistent small weight changes. Removing an incompatible payload can strengthen label retention and directly save communication. The action returns to the reference graph after one step, while rolling-B3—not one-step strong connectivity—protects time-window information flow. Subset actions can combine the three individually favorable formations and may produce a material effect without adding messages. Label-retention, payload, physical, and rolling-connectivity constraints remain hard; H-step consensus becomes a teacher/validation target rather than a false analytic guarantee.

## Evidence boundary

This audit reuses only the frozen v28 primary M24 screen. It may motivate a new action design but cannot relax the failed v28 gate, authorize another opened state, or support validation.
