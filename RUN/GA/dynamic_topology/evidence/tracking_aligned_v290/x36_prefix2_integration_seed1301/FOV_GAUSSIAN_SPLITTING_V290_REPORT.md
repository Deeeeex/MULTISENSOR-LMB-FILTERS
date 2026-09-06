# V290: FoV-aware local update, paired prefix

X36, seed 1301, steps 1--2 of the original 160-step scene; source `7d94cebbaa8c8166d709ecb46b023d4ad3d68f56`. Self-check only, development evidence.

Source qualification: this integration used the uncommitted V290 implementation
over the recorded parent HEAD. That HEAD alone is not a reproducible V290 source
checkpoint; the implementation is frozen in the subsequent source commit.

Only local pre-update density resolution changes; KLA, common labels and V242 routing stay fixed. Reference metrics reuse the saved V242 arm, not the MIL candidate.

| Metric | V242 reference | FoV splitting |
| --- | ---: | ---: |
| eospa | 139.697348582 | 141.828991422 |
| conditionalRmse | 11.831774365 | 12.015034935 |
| countError | 20.805555556 | 21.444444444 |
| representativeDisagreement | 149.479423713 | 148.440758127 |
| worstSensorEospa | 146.843904672 | 146.854579004 |
| attemptedBytes | 677584.000000000 | 2921296.000000000 |
| deliveredBytes | 670880.000000000 | 2886536.000000000 |
| attemptedMessages | 92.000000000 | 92.000000000 |
| deliveredMessages | 91.000000000 | 91.000000000 |
| finiteRmseFraction | 1.000000000 | 1.000000000 |

Common-finite RMSE: 11.831774 -> 12.015035, 72 cells. Matched target identities can differ.
Route-mask differences attempted/delivered: 0/0. Filter runtime 93.1 s; splitter 8.8 s.

Splitting: 2778 input -> 41136 output components across 555 affected label stages; 12786 split operations, maximum 281 components/label, depth 5. Original local posterior cap 20 is unchanged.
Terminal low-weight boundary-leaf conditional mass sum 116.352230 across all label stages; this is not an error bound or discarded mass.

| Formation | Reference E-OSPA | Candidate E-OSPA | Reference RMSE | Candidate RMSE |
| --- | ---: | ---: | ---: | ---: |
| 1 | 146.037161 | 146.844610 | 2.314379 | 2.038723 |
| 2 | 137.840218 | 139.260627 | 12.455896 | 14.389042 |
| 3 | 138.717712 | 140.952877 | 14.574179 | 15.387855 |
| 4 | 138.431519 | 140.677030 | 13.758705 | 14.713358 |
| 5 | 138.958282 | 142.283347 | 12.648488 | 11.154780 |
| 6 | 138.199199 | 140.955458 | 15.238999 | 14.406452 |

Screen evaluated 0; passed 0. Two steps are integration only. No full-episode, held-out or same-fusion static-versus-dynamic conclusion.
