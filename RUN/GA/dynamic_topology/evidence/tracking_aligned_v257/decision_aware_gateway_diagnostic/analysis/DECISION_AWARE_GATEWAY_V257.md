# V257 decision-aware gateway diagnostic

- Analysis commit: `00a6f25db7738a3703bad1e560b42bf8988b57b7`
- Training seeds: `[1302 1303 1304 1307 1308 1309 1310]`
- Windows / actions / safe actions: `42 / 492 / 54`
- Compact signal present: `0`
- Rich information signal present: `0`
- Next decision: `stop-local-gateway-learning-and-change-action-scale-or-horizon`

| Features | Loss | Lambda | Top-safe | Chance | Lift | Positive seeds | Selected | Precision | E | RMSE | Consistency | Pass |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| compact-v256 | multi-output-ridge | 0.0001 | 0.071 | 0.109 | -0.038 | 2/7 | 18/42 | 0.056 | +0.0076 | -0.0308 | +0.0121 | 0 |
| compact-v256 | multi-output-ridge | 0.01 | 0.071 | 0.109 | -0.038 | 2/7 | 19/42 | 0.053 | +0.0051 | -0.1161 | +0.0192 | 0 |
| compact-v256 | multi-output-ridge | 1 | 0.095 | 0.109 | -0.014 | 3/7 | 19/42 | 0.000 | -0.0051 | -0.1969 | +0.0087 | 0 |
| compact-v256 | multi-output-ridge | 100 | 0.143 | 0.109 | +0.033 | 4/7 | 16/42 | 0.125 | +0.0048 | +0.3901 | -0.0375 | 0 |
| compact-v256 | joint-safe-ridge | 0.0001 | 0.119 | 0.109 | +0.010 | 5/7 | 1/42 | 1.000 | +0.0067 | +0.1865 | +0.0382 | 0 |
| compact-v256 | joint-safe-ridge | 0.01 | 0.119 | 0.109 | +0.010 | 5/7 | 1/42 | 1.000 | +0.0067 | +0.1865 | +0.0382 | 0 |
| compact-v256 | joint-safe-ridge | 1 | 0.119 | 0.109 | +0.010 | 5/7 | 1/42 | 1.000 | +0.0067 | +0.1865 | +0.0382 | 0 |
| compact-v256 | joint-safe-ridge | 100 | 0.119 | 0.109 | +0.010 | 5/7 | 0/42 | NaN | +0.0000 | +0.0000 | +0.0000 | 0 |
| rich-pairwise-information-upper-bound | multi-output-ridge | 0.0001 | 0.119 | 0.109 | +0.010 | 4/7 | 22/42 | 0.091 | -0.0126 | +0.1224 | -0.1833 | 0 |
| rich-pairwise-information-upper-bound | multi-output-ridge | 0.01 | 0.095 | 0.109 | -0.014 | 3/7 | 21/42 | 0.095 | -0.0110 | +0.1692 | -0.1578 | 0 |
| rich-pairwise-information-upper-bound | multi-output-ridge | 1 | 0.071 | 0.109 | -0.038 | 2/7 | 20/42 | 0.150 | -0.0046 | -0.0876 | +0.0535 | 0 |
| rich-pairwise-information-upper-bound | multi-output-ridge | 100 | 0.119 | 0.109 | +0.010 | 4/7 | 15/42 | 0.200 | -0.0073 | +0.2121 | +0.0459 | 0 |
| rich-pairwise-information-upper-bound | joint-safe-ridge | 0.0001 | 0.071 | 0.109 | -0.038 | 3/7 | 0/42 | NaN | +0.0000 | +0.0000 | +0.0000 | 0 |
| rich-pairwise-information-upper-bound | joint-safe-ridge | 0.01 | 0.071 | 0.109 | -0.038 | 3/7 | 0/42 | NaN | +0.0000 | +0.0000 | +0.0000 | 0 |
| rich-pairwise-information-upper-bound | joint-safe-ridge | 1 | 0.048 | 0.109 | -0.062 | 2/7 | 0/42 | NaN | +0.0000 | +0.0000 | +0.0000 | 0 |
| rich-pairwise-information-upper-bound | joint-safe-ridge | 100 | 0.119 | 0.109 | +0.010 | 5/7 | 0/42 | NaN | +0.0000 | +0.0000 | +0.0000 | 0 |

## Evidence boundary

V257 uses only the seven V256 training seeds and the same one-arc action outcomes. It compares absolute multi-output regression with joint-safe action ranking under leave-one-seed-out evaluation, and crosses that loss comparison with compact versus exact current-posterior pairwise features. The rich feature mode is an uncosted information upper bound, not a deployable controller. It may only authorize design of a fixed-byte compatibility sketch. Calibration seeds, development holdouts, complete episodes, GNNs and X36 remain sealed.
