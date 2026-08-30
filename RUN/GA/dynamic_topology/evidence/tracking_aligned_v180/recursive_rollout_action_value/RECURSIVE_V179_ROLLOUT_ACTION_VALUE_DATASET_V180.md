# V180 recursive V179-rollout label-action value dataset

- Preset / seed: `x36-formation-fov / 211`
- Feature count: `37`
- Candidate rows / cells: `1378 / 12`
- Shortlist per criterion: `1`

| Split | Cells | Rows | Joint-positive rows | Cells with >=4 safe labels |
|:--|--:|--:|--:|--:|
| training | 6 | 692 | 225 | 6 |
| calibration | 0 | 0 | 0 | 0 |
| heldout | 6 | 686 | 222 | 6 |

| t | F | Receiver | Raw | Shortlist | Safe | Safe labels | Split |
|--:|--:|--:|--:|--:|--:|--:|:--|
| 78 | 5 | 25 | 800 | 113 | 33 | 17 | training |
| 78 | 5 | 26 | 800 | 117 | 38 | 18 | training |
| 78 | 5 | 27 | 800 | 119 | 41 | 19 | training |
| 78 | 5 | 28 | 800 | 115 | 33 | 17 | training |
| 78 | 5 | 29 | 800 | 116 | 42 | 19 | training |
| 78 | 5 | 30 | 800 | 112 | 38 | 19 | training |
| 79 | 5 | 25 | 803 | 119 | 36 | 14 | heldout |
| 79 | 5 | 26 | 803 | 113 | 38 | 17 | heldout |
| 79 | 5 | 27 | 803 | 112 | 40 | 16 | heldout |
| 79 | 5 | 28 | 803 | 112 | 33 | 16 | heldout |
| 79 | 5 | 29 | 803 | 116 | 38 | 19 | heldout |
| 79 | 5 | 30 | 803 | 114 | 37 | 16 | heldout |

## Evidence boundary

V180 is the second opened X36 seed-211 policy-iteration dataset. It reads the pre-side-channel fused posterior and current local source posteriors actually visited by recursive V179 at the twelve F5 t=78/79 cells. Candidate features and shortlisting are present-time and truth-free; current truth scores immediate E-OSPA and RMSE action values. The source rollout and prior learner were developed on seed 211, so this supports method development only, not validation.
