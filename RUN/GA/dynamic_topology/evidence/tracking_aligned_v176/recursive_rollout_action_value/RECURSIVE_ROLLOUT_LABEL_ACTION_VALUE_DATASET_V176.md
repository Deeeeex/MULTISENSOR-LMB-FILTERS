# V176 recursive-rollout label-action value dataset

- Preset / seed: `x36-formation-fov / 211`
- Feature count: `37`
- Candidate rows / cells: `1365 / 12`
- Shortlist per criterion: `1`

| Split | Cells | Rows | Joint-positive rows | Cells with >=4 safe labels |
|:--|--:|--:|--:|--:|
| training | 6 | 692 | 225 | 6 |
| calibration | 0 | 0 | 0 | 0 |
| heldout | 6 | 673 | 224 | 6 |

| t | F | Receiver | Raw | Shortlist | Safe | Safe labels | Split |
|--:|--:|--:|--:|--:|--:|--:|:--|
| 78 | 5 | 25 | 800 | 113 | 33 | 17 | training |
| 78 | 5 | 26 | 800 | 117 | 38 | 18 | training |
| 78 | 5 | 27 | 800 | 119 | 41 | 19 | training |
| 78 | 5 | 28 | 800 | 115 | 33 | 17 | training |
| 78 | 5 | 29 | 800 | 116 | 42 | 19 | training |
| 78 | 5 | 30 | 800 | 112 | 38 | 19 | training |
| 79 | 5 | 25 | 803 | 117 | 34 | 14 | heldout |
| 79 | 5 | 26 | 804 | 112 | 38 | 17 | heldout |
| 79 | 5 | 27 | 804 | 107 | 40 | 16 | heldout |
| 79 | 5 | 28 | 804 | 109 | 35 | 16 | heldout |
| 79 | 5 | 29 | 804 | 116 | 38 | 19 | heldout |
| 79 | 5 | 30 | 804 | 112 | 39 | 18 | heldout |

## Evidence boundary

V176 is an opened X36 seed-211 rollout-state development dataset. It reads the pre-side-channel fused posterior and current local source posteriors actually visited by recursive V169 at the twelve F5 t=78/79 cells. Candidate features and the seven-criterion shortlist use present posterior, topology and FoV summaries only; truth, future measurements and numeric label identifiers are excluded. Current truth scores immediate E-OSPA and matched-position RMSE action values. The source rollout was itself developed on seed 211, so this dataset diagnoses recursive distribution shift and supports policy iteration only; it is not validation or generalization evidence.
