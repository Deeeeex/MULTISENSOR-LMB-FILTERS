# V166 one-hop label-action value dataset

- Preset / seed: `x36-formation-fov / 211`
- Feature count: `37`
- Candidate rows / cells: `3439 / 30`
- Shortlist per criterion: `1`

| Split | Cells | Rows | Joint-positive rows | Cells with >=4 safe labels |
|:--|--:|--:|--:|--:|
| training | 18 | 2087 | 658 | 18 |
| calibration | 6 | 679 | 230 | 6 |
| heldout | 6 | 673 | 232 | 6 |

| t | F | Receiver | Raw | Shortlist | Safe | Safe labels | Split |
|--:|--:|--:|--:|--:|--:|--:|:--|
| 76 | 3 | 13 | 788 | 111 | 30 | 13 | training |
| 76 | 3 | 14 | 789 | 110 | 26 | 14 | training |
| 76 | 3 | 15 | 789 | 111 | 45 | 16 | training |
| 76 | 3 | 16 | 789 | 110 | 28 | 14 | training |
| 76 | 3 | 17 | 789 | 114 | 32 | 16 | training |
| 76 | 3 | 18 | 788 | 114 | 26 | 14 | training |
| 78 | 3 | 13 | 799 | 114 | 33 | 16 | training |
| 78 | 3 | 14 | 799 | 117 | 34 | 18 | training |
| 78 | 3 | 15 | 799 | 118 | 53 | 19 | training |
| 78 | 3 | 16 | 799 | 118 | 37 | 19 | training |
| 78 | 3 | 17 | 799 | 117 | 35 | 19 | training |
| 78 | 3 | 18 | 799 | 116 | 37 | 20 | training |
| 78 | 5 | 25 | 801 | 120 | 35 | 16 | training |
| 78 | 5 | 26 | 801 | 121 | 40 | 18 | training |
| 78 | 5 | 27 | 801 | 121 | 43 | 20 | training |
| 78 | 5 | 28 | 801 | 119 | 38 | 18 | training |
| 78 | 5 | 29 | 801 | 120 | 45 | 19 | training |
| 78 | 5 | 30 | 801 | 116 | 41 | 20 | training |
| 79 | 3 | 13 | 804 | 113 | 32 | 17 | calibration |
| 79 | 3 | 14 | 804 | 116 | 31 | 17 | calibration |
| 79 | 3 | 15 | 804 | 113 | 42 | 19 | calibration |
| 79 | 3 | 16 | 804 | 112 | 57 | 20 | calibration |
| 79 | 3 | 17 | 804 | 111 | 32 | 16 | calibration |
| 79 | 3 | 18 | 804 | 114 | 36 | 15 | calibration |
| 79 | 5 | 25 | 806 | 113 | 32 | 14 | heldout |
| 79 | 5 | 26 | 806 | 112 | 39 | 16 | heldout |
| 79 | 5 | 27 | 806 | 112 | 40 | 17 | heldout |
| 79 | 5 | 28 | 806 | 111 | 36 | 18 | heldout |
| 79 | 5 | 29 | 806 | 113 | 43 | 22 | heldout |
| 79 | 5 | 30 | 806 | 112 | 42 | 19 | heldout |

## Evidence boundary

V166 is an opened X36 seed-211 single-step learnability dataset. Feature rows contain current posterior, physical-neighbor agreement, FoV opportunity, formation-role and payload summaries but exclude numeric label keys, truth, future measurements and alternative-arm state. Current truth supplies immediate complete-label E-OSPA and matched-position RMSE targets. Candidate shortlisting is the union of seven truth-free rankings. Training uses t=76/78 cells, calibration uses t=79 F3 cells, and heldout testing uses t=79 F5 cells. This same-seed temporal/formation split is a learnability screen, not independent validation or generalization evidence.
