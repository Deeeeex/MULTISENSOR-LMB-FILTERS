# V256 pooled expected local gateway ridge

- Analysis commit: `0c0f59fa6abc0702681aebedd2fac1c6ed2a005e`
- Training seeds: `[1302 1303 1304 1307 1308 1309 1310]`
- Rows / features / outcomes: `492 / 32 / 8`
- Deterministic communication-feasible rows: `492/492`
- Actions per window, min / max: `9 / 12`
- Selected ridge lambda: `100`
- Mean tracking skill over seed-blind mean: `-3.636%`
- Receiver-RMSE skill over seed-blind mean: `-9.076%`
- Calibration authorized: `0`
- Next decision: `stop-feature-conditioned-v256-before-calibration`

| Lambda | Tracking normalized MSE | Baseline | Skill | Receiver-RMSE MSE | Baseline | Skill |
|--:|--:|--:|--:|--:|--:|--:|
| 0.0001 | 1.36766 | 1.25169 | -9.265% | 1.58252 | 1.35061 | -17.170% |
| 0.01 | 1.36641 | 1.25169 | -9.165% | 1.57966 | 1.35061 | -16.959% |
| 1 | 1.34581 | 1.25169 | -7.519% | 1.53816 | 1.35061 | -13.886% |
| 100 | 1.29720 | 1.25169 | -3.636% | 1.47319 | 1.35061 | -9.076% |

## Selected cross-seed outcome errors

| Outcome | Model MSE | Mean baseline MSE | Skill |
|:--|--:|--:|--:|
| eospa_gain_percent | 1.04950 | 1.02500 | -2.390% |
| rmse_gain_percent | 1.15944 | 1.14684 | -1.098% |
| consistency_gain_percent | 1.61652 | 1.52071 | -6.300% |
| total_byte_saving_percent | 1.01497 | 1.04605 | +2.971% |
| minimum_formation_eospa_gain_percent | 1.01895 | 1.04643 | +2.626% |
| minimum_formation_rmse_gain_percent | 1.72225 | 1.64896 | -4.444% |
| receiver_formation_eospa_gain_percent | 1.04057 | 1.02329 | -1.689% |
| receiver_formation_rmse_gain_percent | 1.47319 | 1.35061 | -9.076% |

## Evidence boundary

V256 estimates the conditional paired average effect of one local gateway replacement from seven M24 training seeds. Ridge regularization is selected by leave-one-seed-out standardized prediction error over seven tracking outcomes; attempted-byte prediction is diagnostic because admission uses the deterministic V255 communication-credit projection against the current physical V240 two-input causal route. The projection estimates all three posterior pages from current payload sizes and charges one controller exchange; it retains at least 80 percent of the estimated dense-to-V242 saving. Seeds 1311--1312 are used only for one-sided empirical seed-block margins after the feature representation and ridge penalty are frozen. Seed 1306 is opened once for method evaluation and seed 1305 remains untouched for a complete episode. The small calibration set does not support a formal finite-sample coverage claim. No result authorizes a GNN, X36 tuning or paper claim.
