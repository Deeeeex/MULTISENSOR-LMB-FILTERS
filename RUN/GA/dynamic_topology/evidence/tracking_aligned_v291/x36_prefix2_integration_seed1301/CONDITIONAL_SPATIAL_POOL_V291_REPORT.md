# V291: conditional spatial-pooling control

X36 seed 1301, original 160-step scene prefix 1--2; source `bf3dca09bc1bd3f7a95e87056bc570e613cf142c`. Self-check only.

Not standard LMB-KLA/MIL, not a reproduced Uney method and not new routing. Compared with V288, retain arithmetic existence, zero extension and existence-weighted conditional weights; replace only the spatial pooling operator, including its documented numerical approximation. Recursive inputs can subsequently differ.

| Metric | V242 KLA | V288 MIL | Conditional geometric |
| --- | ---: | ---: | ---: |
| eospa | 139.697348582 | 139.270652624 | 139.289018877 |
| conditionalRmse | 11.831774365 | 13.737501642 | 12.337711274 |
| countError | 20.805555556 | 20.666666667 | 20.680555556 |
| representativeDisagreement | 149.479423713 | 147.362761910 | 147.363946206 |
| worstSensorEospa | 146.843904672 | 146.843245959 | 146.843584328 |
| attemptedBytes | 677584.000000000 | 964696.000000000 | 688168.000000000 |
| deliveredBytes | 670880.000000000 | 957992.000000000 | 681464.000000000 |
| attemptedMessages | 92.000000000 | 92.000000000 | 92.000000000 |
| deliveredMessages | 91.000000000 | 91.000000000 | 91.000000000 |
| finiteRmseFraction | 1.000000000 | 1.000000000 | 1.000000000 |

Common-finite RMSE versus KLA: 11.831774 -> 12.337711 (72 cells). Versus MIL: 13.737502 -> 12.337711 (72 cells). Matching identities can differ.
Route-mask differences attempted/delivered: 0/0; filter 21.5 s.

| Formation | KLA E | V291 E | KLA RMSE | V291 RMSE |
| --- | ---: | ---: | ---: | ---: |
| 1 | 146.037161 | 146.036920 | 2.314379 | 2.526120 |
| 2 | 137.840218 | 137.857680 | 12.455896 | 12.979420 |
| 3 | 138.717712 | 137.617861 | 14.574179 | 16.808392 |
| 4 | 138.431519 | 137.871520 | 13.758705 | 13.228373 |
| 5 | 138.958282 | 138.686424 | 12.648488 | 12.436400 |
| 6 | 138.199199 | 137.663709 | 15.238999 | 16.047563 |

Screen evaluated 0; passed 0. Two steps are integration only. No automatic full/M24 extension and no claim of across-seed significance.
