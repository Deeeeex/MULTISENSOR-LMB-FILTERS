# V200 V199 F3 label-action localization

- Preset / seed: `x36-formation-fov / 211`
- Feature count: `37`
- Candidate rows / cells: `2034 / 18`
- Shortlist per criterion: `1`

| Split | Cells | Rows | Joint-positive rows | Cells with >=4 safe labels |
|:--|--:|--:|--:|--:|
| opened | 18 | 2034 | 652 | 18 |
| unused | 0 | 0 | 0 | 0 |
| unused | 0 | 0 | 0 | 0 |

| t | F | Receiver | Raw | Shortlist | Safe | Safe labels | Split |
|--:|--:|--:|--:|--:|--:|--:|:--|
| 76 | 3 | 13 | 788 | 111 | 30 | 13 | opened |
| 76 | 3 | 14 | 789 | 110 | 27 | 14 | opened |
| 76 | 3 | 15 | 789 | 111 | 44 | 16 | opened |
| 76 | 3 | 16 | 789 | 110 | 29 | 14 | opened |
| 76 | 3 | 17 | 789 | 114 | 32 | 16 | opened |
| 76 | 3 | 18 | 788 | 114 | 26 | 14 | opened |
| 78 | 3 | 13 | 798 | 112 | 31 | 15 | opened |
| 78 | 3 | 14 | 798 | 114 | 34 | 18 | opened |
| 78 | 3 | 15 | 798 | 117 | 54 | 19 | opened |
| 78 | 3 | 16 | 798 | 116 | 36 | 19 | opened |
| 78 | 3 | 17 | 798 | 114 | 34 | 19 | opened |
| 78 | 3 | 18 | 798 | 114 | 38 | 20 | opened |
| 79 | 3 | 13 | 800 | 112 | 37 | 17 | opened |
| 79 | 3 | 14 | 800 | 115 | 32 | 14 | opened |
| 79 | 3 | 15 | 800 | 113 | 42 | 18 | opened |
| 79 | 3 | 16 | 800 | 112 | 56 | 19 | opened |
| 79 | 3 | 17 | 800 | 112 | 33 | 15 | opened |
| 79 | 3 | 18 | 800 | 113 | 37 | 15 | opened |

## Evidence boundary

V200 scores current physical-neighbor complete-label replacements on the 18 F3 receiver-time cells at t=76/78/79 from the already completed V199 rollout. Candidate features and shortlisting use only current posterior, geometry and source information; current truth supplies immediate E-OSPA and matched-RMSE targets after selection. All cells are opened development attribution, not training, validation, generalization or a deployable policy.
