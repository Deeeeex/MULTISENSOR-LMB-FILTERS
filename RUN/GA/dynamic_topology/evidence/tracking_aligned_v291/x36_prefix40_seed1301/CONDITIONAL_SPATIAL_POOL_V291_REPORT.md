# V291: conditional spatial-pooling control

X36 seed 1301, original 160-step scene prefix 1--40; source `bf3dca09bc1bd3f7a95e87056bc570e613cf142c`. Self-check only.

Not standard LMB-KLA/MIL, not a reproduced Uney method and not new routing. Compared with V288, retain arithmetic existence, zero extension and existence-weighted conditional weights; replace only the spatial pooling operator, including its documented numerical approximation. Recursive inputs can subsequently differ.

| Metric | V242 KLA | V288 MIL | Conditional geometric |
| --- | ---: | ---: | ---: |
| eospa | 135.180029883 | 132.617636938 | 132.339040569 |
| conditionalRmse | 8.425317479 | 19.735673677 | 8.524261702 |
| countError | 19.501388889 | 18.631944444 | 18.684722222 |
| representativeDisagreement | 144.669698910 | 140.162516091 | 139.561394004 |
| worstSensorEospa | 137.719448351 | 134.289700680 | 134.115377354 |
| attemptedBytes | 18435344.000000000 | 59867264.000000000 | 19033448.000000000 |
| deliveredBytes | 17667904.000000000 | 57266464.000000000 | 18251752.000000000 |
| attemptedMessages | 1840.000000000 | 1840.000000000 | 1840.000000000 |
| deliveredMessages | 1766.000000000 | 1766.000000000 | 1766.000000000 |
| finiteRmseFraction | 1.000000000 | 1.000000000 | 1.000000000 |

Common-finite RMSE versus KLA: 8.425317 -> 8.524262 (1440 cells). Versus MIL: 19.735674 -> 8.524262 (1440 cells). Matching identities can differ.
Route-mask differences attempted/delivered: 0/0; filter 634.7 s.

| Formation | KLA E | V291 E | KLA RMSE | V291 RMSE |
| --- | ---: | ---: | ---: | ---: |
| 1 | 131.108544 | 130.082536 | 8.987466 | 9.826667 |
| 2 | 133.828655 | 130.913777 | 7.356471 | 6.840342 |
| 3 | 136.465141 | 133.096662 | 8.272033 | 8.285258 |
| 4 | 136.742390 | 132.789520 | 8.547453 | 9.681888 |
| 5 | 137.278990 | 133.243905 | 8.192803 | 8.469961 |
| 6 | 135.656459 | 133.907843 | 9.195678 | 8.041456 |

Screen evaluated 1; passed 0. Two steps are integration only. No automatic full/M24 extension and no claim of across-seed significance.
