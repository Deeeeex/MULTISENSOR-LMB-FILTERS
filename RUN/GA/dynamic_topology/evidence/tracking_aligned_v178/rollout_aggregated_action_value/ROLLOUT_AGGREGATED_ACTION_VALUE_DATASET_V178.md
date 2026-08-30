# V178 rollout-aggregated label-action dataset

- Features / rows / cells: `37 / 4131 / 36`
- Sources: `V166 static + V176 recursive rollout`

| Split | Source | Cells | Rows | Joint-positive | Cells with safe actions |
|:--|:--|--:|--:|--:|--:|
| training | V166-static | 18 | 2087 | 648 | 18 |
| training | V176-rollout | 6 | 692 | 225 | 6 |
| calibration | V166-static | 6 | 679 | 224 | 6 |
| calibration | V176-rollout | 0 | 0 | 0 | 0 |
| heldout | V166-static | 0 | 0 | 0 | 0 |
| heldout | V176-rollout | 6 | 673 | 224 | 6 |

## Evidence boundary

V178 is an opened seed-211 policy-iteration dataset. Training joins the V166 t=76/78 static-snapshot cells with the V176 t=78 states actually visited by recursive V169. Hyperparameter and threshold selection may use only the V166 F3 t=79 calibration cells. The V176 F5 t=79 rollout cells remain grouped heldout until selection is frozen. Features use only present posterior, topology and FoV summaries; current truth supplies immediate action targets. Since all states come from opened X36 seed 211 and the rollout was induced by a previously developed policy, this supports recursive method development only, not validation or generalization.
