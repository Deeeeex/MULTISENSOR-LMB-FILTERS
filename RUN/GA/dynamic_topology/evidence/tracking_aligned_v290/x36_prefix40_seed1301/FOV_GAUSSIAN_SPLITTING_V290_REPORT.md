# V290: FoV-aware local update, paired prefix

X36, seed 1301, steps 1--40 of the original 160-step scene; source `702f84d85da0dc935f1c3dfaa75061e454e101d8`. Self-check only, development evidence.

Only local pre-update density resolution changes; KLA, common labels and V242 routing stay fixed. Reference metrics reuse the saved V242 arm, not the MIL candidate.

| Metric | V242 reference | FoV splitting |
| --- | ---: | ---: |
| eospa | 135.180029883 | 136.820304846 |
| conditionalRmse | 8.425317479 | 8.725533078 |
| countError | 19.501388889 | 19.975694444 |
| representativeDisagreement | 144.669698910 | 144.634321193 |
| worstSensorEospa | 137.719448351 | 140.089920583 |
| attemptedBytes | 18435344.000000000 | 67000712.000000000 |
| deliveredBytes | 17667904.000000000 | 64399936.000000000 |
| attemptedMessages | 1840.000000000 | 1840.000000000 |
| deliveredMessages | 1766.000000000 | 1766.000000000 |
| finiteRmseFraction | 1.000000000 | 1.000000000 |

Common-finite RMSE: 8.425317 -> 8.725533, 1440 cells. Matched target identities can differ.
Route-mask differences attempted/delivered: 0/0. Filter runtime 1575.7 s; splitter 100.6 s.

Splitting: 64048 input -> 483286 output components across 14772 affected label stages; 139746 split operations, maximum 281 components/label, depth 5. Original local posterior cap 20 is unchanged.
Terminal low-weight boundary-leaf conditional mass sum 900.680272 across all label stages; this is not an error bound or discarded mass.

| Formation | Reference E-OSPA | Candidate E-OSPA | Reference RMSE | Candidate RMSE |
| --- | ---: | ---: | ---: | ---: |
| 1 | 131.108544 | 136.201463 | 8.987466 | 8.424171 |
| 2 | 133.828655 | 134.498915 | 7.356471 | 7.605888 |
| 3 | 136.465141 | 139.210933 | 8.272033 | 9.006399 |
| 4 | 136.742390 | 137.548235 | 8.547453 | 9.404537 |
| 5 | 137.278990 | 137.513859 | 8.192803 | 8.333141 |
| 6 | 135.656459 | 135.948424 | 9.195678 | 9.579063 |

Screen evaluated 1; passed 0. Two steps are integration only. No full-episode, held-out or same-fusion static-versus-dynamic conclusion.
