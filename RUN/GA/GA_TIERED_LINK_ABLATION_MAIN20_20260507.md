# GA Tiered Link Ablation Main20 Summary
Generated: 2026-05-07 02:09:12
Source reports:
- `RUN/GA/GA_TIERED_LINK_ABLATION_N5_SEED1_20260507_020759.md`
- `RUN/GA/GA_TIERED_LINK_ABLATION_N5_SEED6_20260507_020740.md`
- `RUN/GA/GA_TIERED_LINK_ABLATION_N5_SEED11_20260507_020838.md`
- `RUN/GA/GA_TIERED_LINK_ABLATION_N5_SEED16_20260507_020811.md`

## Run Config
- Arms: fixed weights -> +covariance -> +link quality -> +existence confidence -> +structure-aware decoupled KLA
- Trial seeds: 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21
- Scenario: tiered heterogeneous packet loss, communication level 2, global delivered-measurement cap 80, fixed link loss, pDrop levels [0, 0.1, 0.2, 0.5] with counts [1, 4, 1, 2].

## Per-Trial Consensus Metrics
| Seed | Arm | OSPA | RMSE | Cardinality |
|-----:|:----|-----:|-----:|------------:|
| 2 | fixed weights | 2.590531 | 2.268101 | 0.868750 |
| 2 | +covariance | 2.243220 | 1.774557 | 0.608750 |
| 2 | +link quality | 1.909508 | 1.621662 | 0.242500 |
| 2 | +existence confidence | 1.911733 | 1.681976 | 0.238750 |
| 2 | +structure-aware decoupled KLA | 1.892404 | 1.611468 | 0.241250 |
| 3 | fixed weights | 2.623249 | 3.319849 | 0.761250 |
| 3 | +covariance | 2.295843 | 2.943444 | 0.610000 |
| 3 | +link quality | 1.941786 | 1.634121 | 0.271250 |
| 3 | +existence confidence | 1.955997 | 1.546617 | 0.275000 |
| 3 | +structure-aware decoupled KLA | 1.928044 | 1.541570 | 0.267500 |
| 4 | fixed weights | 2.681097 | 2.507659 | 0.871250 |
| 4 | +covariance | 2.137026 | 1.754357 | 0.537500 |
| 4 | +link quality | 1.853284 | 1.495404 | 0.236250 |
| 4 | +existence confidence | 1.844136 | 1.482744 | 0.233750 |
| 4 | +structure-aware decoupled KLA | 1.835859 | 1.487550 | 0.237500 |
| 5 | fixed weights | 2.398507 | 2.920391 | 0.625000 |
| 5 | +covariance | 2.017490 | 3.202154 | 0.371250 |
| 5 | +link quality | 1.774282 | 2.228815 | 0.192500 |
| 5 | +existence confidence | 1.772742 | 2.224507 | 0.188750 |
| 5 | +structure-aware decoupled KLA | 1.764445 | 2.213187 | 0.191250 |
| 6 | fixed weights | 2.826942 | 2.497009 | 1.267500 |
| 6 | +covariance | 2.363987 | 2.380371 | 0.820000 |
| 6 | +link quality | 1.909992 | 2.024725 | 0.283750 |
| 6 | +existence confidence | 1.904559 | 1.923063 | 0.285000 |
| 6 | +structure-aware decoupled KLA | 1.890466 | 1.894265 | 0.283750 |
| 7 | fixed weights | 2.541049 | 2.575987 | 0.747500 |
| 7 | +covariance | 2.101247 | 2.137851 | 0.448750 |
| 7 | +link quality | 1.842867 | 1.861863 | 0.210000 |
| 7 | +existence confidence | 1.825836 | 1.828658 | 0.205000 |
| 7 | +structure-aware decoupled KLA | 1.823612 | 1.828964 | 0.205000 |
| 8 | fixed weights | 2.386825 | 2.412256 | 0.563750 |
| 8 | +covariance | 2.015325 | 2.036865 | 0.400000 |
| 8 | +link quality | 1.750055 | 1.509791 | 0.178750 |
| 8 | +existence confidence | 1.779008 | 1.507649 | 0.185000 |
| 8 | +structure-aware decoupled KLA | 1.762642 | 1.481461 | 0.185000 |
| 9 | fixed weights | 2.456704 | 2.596131 | 0.663750 |
| 9 | +covariance | 1.970587 | 2.020098 | 0.382500 |
| 9 | +link quality | 1.763240 | 1.540145 | 0.172500 |
| 9 | +existence confidence | 1.768565 | 1.502216 | 0.175000 |
| 9 | +structure-aware decoupled KLA | 1.756928 | 1.546963 | 0.172500 |
| 10 | fixed weights | 2.571992 | 3.155163 | 0.736250 |
| 10 | +covariance | 2.231084 | 2.152015 | 0.563750 |
| 10 | +link quality | 1.837096 | 2.446852 | 0.218750 |
| 10 | +existence confidence | 1.845660 | 2.469694 | 0.220000 |
| 10 | +structure-aware decoupled KLA | 1.823264 | 2.505816 | 0.215000 |
| 11 | fixed weights | 2.175224 | 2.634355 | 0.416250 |
| 11 | +covariance | 1.923264 | 2.047288 | 0.347500 |
| 11 | +link quality | 1.818784 | 1.608326 | 0.205000 |
| 11 | +existence confidence | 1.822877 | 1.594128 | 0.205000 |
| 11 | +structure-aware decoupled KLA | 1.831138 | 1.611837 | 0.212500 |
| 12 | fixed weights | 2.235333 | 2.348212 | 0.482500 |
| 12 | +covariance | 2.005310 | 2.061706 | 0.396250 |
| 12 | +link quality | 1.803088 | 1.857358 | 0.223750 |
| 12 | +existence confidence | 1.804221 | 1.894255 | 0.220000 |
| 12 | +structure-aware decoupled KLA | 1.797235 | 1.885002 | 0.225000 |
| 13 | fixed weights | 2.380325 | 2.213801 | 0.656250 |
| 13 | +covariance | 1.931672 | 1.964170 | 0.347500 |
| 13 | +link quality | 1.777604 | 1.479052 | 0.277500 |
| 13 | +existence confidence | 1.776705 | 1.527480 | 0.273750 |
| 13 | +structure-aware decoupled KLA | 1.786939 | 1.575583 | 0.281250 |
| 14 | fixed weights | 2.363292 | 2.665071 | 0.552500 |
| 14 | +covariance | 2.058895 | 2.387448 | 0.357500 |
| 14 | +link quality | 1.857518 | 1.812120 | 0.225000 |
| 14 | +existence confidence | 1.854604 | 1.747155 | 0.221250 |
| 14 | +structure-aware decoupled KLA | 1.850337 | 1.788203 | 0.222500 |
| 15 | fixed weights | 2.147298 | 2.230920 | 0.382500 |
| 15 | +covariance | 1.946851 | 1.846261 | 0.310000 |
| 15 | +link quality | 1.804698 | 1.606526 | 0.245000 |
| 15 | +existence confidence | 1.800481 | 1.606367 | 0.246250 |
| 15 | +structure-aware decoupled KLA | 1.798971 | 1.592700 | 0.247500 |
| 16 | fixed weights | 2.048554 | 2.956503 | 0.333750 |
| 16 | +covariance | 1.888464 | 2.138081 | 0.310000 |
| 16 | +link quality | 1.792985 | 2.192421 | 0.217500 |
| 16 | +existence confidence | 1.779075 | 2.164613 | 0.213750 |
| 16 | +structure-aware decoupled KLA | 1.765436 | 2.136723 | 0.210000 |
| 17 | fixed weights | 2.600917 | 2.376823 | 0.802500 |
| 17 | +covariance | 2.123460 | 1.822961 | 0.482500 |
| 17 | +link quality | 1.846428 | 1.442536 | 0.260000 |
| 17 | +existence confidence | 1.872548 | 1.446483 | 0.265000 |
| 17 | +structure-aware decoupled KLA | 1.863439 | 1.430953 | 0.268750 |
| 18 | fixed weights | 2.530244 | 2.232345 | 0.762500 |
| 18 | +covariance | 2.228898 | 2.359800 | 0.537500 |
| 18 | +link quality | 1.880047 | 1.767543 | 0.233750 |
| 18 | +existence confidence | 1.881488 | 1.777117 | 0.232500 |
| 18 | +structure-aware decoupled KLA | 1.873030 | 1.746324 | 0.237500 |
| 19 | fixed weights | 2.440243 | 2.295666 | 0.741250 |
| 19 | +covariance | 1.949824 | 2.437999 | 0.398750 |
| 19 | +link quality | 1.725516 | 1.706628 | 0.208750 |
| 19 | +existence confidence | 1.713947 | 1.653472 | 0.212500 |
| 19 | +structure-aware decoupled KLA | 1.703659 | 1.650134 | 0.208750 |
| 20 | fixed weights | 2.498598 | 2.662959 | 0.777500 |
| 20 | +covariance | 2.100110 | 1.792176 | 0.487500 |
| 20 | +link quality | 1.866973 | 1.688004 | 0.248750 |
| 20 | +existence confidence | 1.844706 | 1.681704 | 0.246250 |
| 20 | +structure-aware decoupled KLA | 1.851685 | 1.651547 | 0.253750 |
| 21 | fixed weights | 2.462637 | 2.808408 | 0.576250 |
| 21 | +covariance | 2.116350 | 2.094606 | 0.441250 |
| 21 | +link quality | 1.838601 | 1.759714 | 0.262500 |
| 21 | +existence confidence | 1.837733 | 1.812616 | 0.267500 |
| 21 | +structure-aware decoupled KLA | 1.829428 | 1.815996 | 0.266250 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.447978 +/- 0.191065 | [2.358558, 2.537398] | 20 |
| fixed weights | RMSE | 2.583880 +/- 0.315832 | [2.436068, 2.731693] | 20 |
| fixed weights | Cardinality | 0.679437 +/- 0.208768 | [0.581732, 0.777143] | 20 |
| +covariance | OSPA | 2.082445 +/- 0.135741 | [2.018917, 2.145973] | 20 |
| +covariance | RMSE | 2.167710 +/- 0.375386 | [1.992026, 2.343394] | 20 |
| +covariance | Cardinality | 0.457937 +/- 0.126652 | [0.398663, 0.517212] | 20 |
| +link quality | OSPA | 1.829718 +/- 0.056857 | [1.803108, 1.856327] | 20 |
| +link quality | RMSE | 1.764180 +/- 0.273490 | [1.636184, 1.892176] | 20 |
| +link quality | Cardinality | 0.230687 +/- 0.031407 | [0.215989, 0.245386] | 20 |
| +existence confidence | OSPA | 1.829831 +/- 0.058066 | [1.802656, 1.857006] | 20 |
| +existence confidence | RMSE | 1.753626 +/- 0.272847 | [1.625931, 1.881321] | 20 |
| +existence confidence | Cardinality | 0.230500 +/- 0.031676 | [0.215675, 0.245325] | 20 |
| +structure-aware decoupled KLA | OSPA | 1.821448 +/- 0.055189 | [1.795619, 1.847277] | 20 |
| +structure-aware decoupled KLA | RMSE | 1.749812 +/- 0.274055 | [1.621552, 1.878073] | 20 |
| +structure-aware decoupled KLA | Cardinality | 0.231625 +/- 0.032213 | [0.216549, 0.246701] | 20 |
