# Paper-facing N=50 synthesis

Generated at: 2026-05-28 21:00:31

This file reorders the batch outputs into the exact paper-table logic. It does not rerun simulations.

## Source mapping

| Paper table | Source summary | Rows used |
|:--|:--|:--|
| Main consensus/local | 01 + 03 | Fixed/FID/Balanced/Cardinality from 01; PD/FI from 03 |
| Runtime cost | 01 + 03 | Fixed/FID/Balanced/Cardinality from 01; PD/FI from the direct dynamic-weighting probe |
| Factor ablation | 02 + 01 | First five backbone rows from 02; Cardinality-critical from 01 |
| Ideal-support | 04 | Reordered as Ordinary, Balanced, FID-FIA, Cardinality-critical |
| Communication sensitivity | 05-08 | Fixed/Balanced/Cardinality-critical per communication level |

## Main tiered-drop table

| Arm | OSPA err. | Loc. disag. | Card. disp. | OSPA std | Loc. std | Card. std | Source |
|:--|--:|--:|--:|--:|--:|--:|:--|
| Fixed Metropolis | 2.468973 | 2.326034 | 0.716025 | 0.219981 | 0.353065 | 0.223795 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/01_tiered_main_fidfia_n50_seed1.mat |
| PD-weighted GA | 2.177337 | 1.994628 | 0.588025 | 0.161465 | 0.232503 | 0.170389 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/03_tiered_pd_fi_baselines_n50_seed1.mat |
| FI-weighted GA | 2.029572 | 1.869792 | 0.440850 | 0.128282 | 0.220981 | 0.126942 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/03_tiered_pd_fi_baselines_n50_seed1.mat |
| FID-FIA baseline | 1.817913 | 1.642846 | 0.122950 | 0.056010 | 0.109008 | 0.022854 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/01_tiered_main_fidfia_n50_seed1.mat |
| Balanced mode | 1.779218 | 1.522191 | 0.187675 | 0.072856 | 0.157069 | 0.031189 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/01_tiered_main_fidfia_n50_seed1.mat |
| Cardinality-critical mode | 1.677524 | 1.546107 | 0.063200 | 0.049899 | 0.157649 | 0.017041 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/01_tiered_main_fidfia_n50_seed1.mat |

| Arm | E-OSPA | RMSE | CardErr | Source |
|:--|--:|--:|--:|:--|
| Fixed Metropolis | 2.862938 | 1.649569 | 1.455125 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/01_tiered_main_fidfia_n50_seed1.mat |
| PD-weighted GA | 2.735723 | 1.562807 | 1.254925 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/03_tiered_pd_fi_baselines_n50_seed1.mat |
| FI-weighted GA | 2.495536 | 1.547675 | 0.984750 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/03_tiered_pd_fi_baselines_n50_seed1.mat |
| FID-FIA baseline | 2.184698 | 1.734381 | 0.388050 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/01_tiered_main_fidfia_n50_seed1.mat |
| Balanced mode | 2.334915 | 1.605910 | 0.578775 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/01_tiered_main_fidfia_n50_seed1.mat |
| Cardinality-critical mode | 2.019842 | 1.720931 | 0.223700 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/01_tiered_main_fidfia_n50_seed1.mat |

### Runtime rows used by the current paper

The PD/FI rows are from the direct dynamic-weighting probe under the same scenario and seed set. Their relative runtime uses that probe's paired Fixed Metropolis denominator.

| Arm | Runtime (s) | Runtime/step (s) | Relative runtime | Runtime std | Source |
|:--|--:|--:|--:|--:|:--|
| Fixed Metropolis | 52.122628 | 0.521226 | 1.000000 | 7.932188 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/01_tiered_main_fidfia_n50_seed1.mat |
| PD-weighted GA | 61.668471 | 0.616685 | 1.233000 | 5.376481 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/Del_GA_TIERED_LINK_ABLATION_N50_SEED1_20260528_092545.md |
| FI-weighted GA | 62.810458 | 0.628105 | 1.254000 | 6.767913 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/Del_GA_TIERED_LINK_ABLATION_N50_SEED1_20260528_092545.md |
| FID-FIA baseline | 147.673755 | 1.476738 | 2.833199 | 23.956635 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/01_tiered_main_fidfia_n50_seed1.mat |
| Balanced mode | 56.378137 | 0.563781 | 1.081644 | 9.626476 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/01_tiered_main_fidfia_n50_seed1.mat |
| Cardinality-critical mode | 155.913438 | 1.559134 | 2.991281 | 18.219890 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/01_tiered_main_fidfia_n50_seed1.mat |

## Factor ablation table

| Arm | OSPA err. | Loc. disag. | Card. disp. | OSPA std | Loc. std | Card. std | Source |
|:--|--:|--:|--:|--:|--:|--:|:--|
| Fixed Metropolis | 2.468973 | 2.326034 | 0.716025 | 0.219981 | 0.353065 | 0.223795 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/02_tiered_factor_ablation_n50_seed1.mat |
| Covariance-only adaptive | 2.090649 | 1.950889 | 0.465850 | 0.144677 | 0.252957 | 0.134228 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/02_tiered_factor_ablation_n50_seed1.mat |
| Covariance-link adaptive | 1.788038 | 1.524995 | 0.188150 | 0.071415 | 0.155955 | 0.031280 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/02_tiered_factor_ablation_n50_seed1.mat |
| Three-factor adaptive backbone | 1.791239 | 1.534119 | 0.187875 | 0.073462 | 0.161038 | 0.031268 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/02_tiered_factor_ablation_n50_seed1.mat |
| Balanced mode | 1.779218 | 1.522191 | 0.187675 | 0.072856 | 0.157069 | 0.031189 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/02_tiered_factor_ablation_n50_seed1.mat |
| Cardinality-critical mode | 1.677524 | 1.546107 | 0.063200 | 0.049899 | 0.157649 | 0.017041 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/01_tiered_main_fidfia_n50_seed1.mat |

## Ideal-communication support table

| Arm | OSPA err. | Loc. disag. | Card. disp. | OSPA std | Loc. std | Card. std | Source |
|:--|--:|--:|--:|--:|--:|--:|:--|
| Ordinary GA | 1.634289 | 1.381434 | 0.090125 | 0.040164 | 0.040960 | 0.020982 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/04_ideal_fidfia_n50_seed1.mat |
| Balanced mode | 1.427058 | 1.201835 | 0.071300 | 0.035445 | 0.030438 | 0.017489 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/04_ideal_fidfia_n50_seed1.mat |
| FID-FIA baseline | 1.534235 | 1.332864 | 0.057400 | 0.035889 | 0.034054 | 0.012797 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/04_ideal_fidfia_n50_seed1.mat |
| Cardinality-critical mode | 1.432865 | 1.302686 | 0.048650 | 0.033613 | 0.165901 | 0.011146 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/04_ideal_fidfia_n50_seed1.mat |

| Arm | E-OSPA | RMSE | CardErr | Source |
|:--|--:|--:|--:|:--|
| Ordinary GA | 1.908084 | 1.441941 | 0.281625 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/04_ideal_fidfia_n50_seed1.mat |
| Balanced mode | 1.838671 | 1.374600 | 0.270750 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/04_ideal_fidfia_n50_seed1.mat |
| FID-FIA baseline | 1.880574 | 1.499905 | 0.225500 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/04_ideal_fidfia_n50_seed1.mat |
| Cardinality-critical mode | 1.765967 | 1.470315 | 0.195350 | /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/mc50_20260527_172137/04_ideal_fidfia_n50_seed1.mat |

## Communication-level sensitivity table

| Level | Fixed OSPA | Balanced OSPA | Fixed loc. disag. | Balanced loc. disag. | Fixed card. disp. | Balanced card. disp. |
|--:|--:|--:|--:|--:|--:|--:|
| 0 | 1.634289 | 1.427058 | 1.381434 | 1.201835 | 0.090125 | 0.071300 |
| 1 | pending | pending | pending | pending | pending | pending |
| 2 | pending | pending | pending | pending | pending | pending |
| 3 | pending | pending | pending | pending | pending | pending |
