# Reviewer-requested baseline comparison

- Seeds: 2--51 (50 deterministic paired trials)
- OSPA/GOSPA: c=5, p=2; GOSPA alpha=2
- Ground space: complete extracted kinematic state vector
- Core source: `/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS-inffus/RUN/GA/gospa_validation/gospa_core_n50_summary.mat`
- Adaptation source: `/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS-inffus/RUN/GA/reviewer_baselines_validation/reviewer_baselines_n50_summary.mat`
- Zheng/Gao rows are adaptations, not exact source implementations.

| Arm | OSPA | GOSPA | Loc. disag. | Card. disp. | E-OSPA | RMSE | CardErr |
|:----|-----:|------:|------------:|------------:|-------:|-----:|--------:|
| Fixed Metropolis | 2.469 +/- 0.220 | 4.780 +/- 0.245 | 2.326 +/- 0.353 | 0.716 +/- 0.224 | 2.863 +/- 0.124 | 1.650 +/- 0.078 | 1.455 +/- 0.242 |
| Zheng-style subdensity GA-LMB adaptation | 3.577 +/- 0.241 | 5.008 +/- 0.357 | 13.104 +/- 3.144 | 0.969 +/- 0.226 | 4.243 +/- 0.244 | 2.906 +/- 0.732 | 4.202 +/- 0.550 |
| Gao-style local-trust GA-LMB adaptation | 3.332 +/- 0.149 | 6.203 +/- 0.152 | 4.048 +/- 0.581 | 1.438 +/- 0.210 | 3.092 +/- 0.108 | 2.184 +/- 0.238 | 2.002 +/- 0.247 |
| Balanced mode | 1.779 +/- 0.073 | 3.909 +/- 0.124 | 1.522 +/- 0.157 | 0.188 +/- 0.031 | 2.335 +/- 0.072 | 1.606 +/- 0.050 | 0.579 +/- 0.067 |
| Cardinality-critical mode | 1.678 +/- 0.050 | 3.904 +/- 0.114 | 1.546 +/- 0.158 | 0.063 +/- 0.017 | 2.020 +/- 0.047 | 1.721 +/- 0.161 | 0.224 +/- 0.029 |

## Paired network-metric sign tests versus Fixed Metropolis

| Arm | Metric | Improved trials | Exact two-sided p |
|:----|:-------|----------------:|------------------:|
| Zheng-style subdensity GA-LMB adaptation | OSPA | 0/50 | 1.78e-15 |
| Zheng-style subdensity GA-LMB adaptation | GOSPA | 14/50 | 0.0026 |
| Zheng-style subdensity GA-LMB adaptation | Loc. disag. | 0/50 | 1.78e-15 |
| Zheng-style subdensity GA-LMB adaptation | Card. disp. | 6/50 | 3.24e-08 |
| Gao-style local-trust GA-LMB adaptation | OSPA | 0/50 | 1.78e-15 |
| Gao-style local-trust GA-LMB adaptation | GOSPA | 0/50 | 1.78e-15 |
| Gao-style local-trust GA-LMB adaptation | Loc. disag. | 0/50 | 1.78e-15 |
| Gao-style local-trust GA-LMB adaptation | Card. disp. | 0/50 | 1.78e-15 |
| Balanced mode | OSPA | 50/50 | 1.78e-15 |
| Balanced mode | GOSPA | 50/50 | 1.78e-15 |
| Balanced mode | Loc. disag. | 50/50 | 1.78e-15 |
| Balanced mode | Card. disp. | 50/50 | 1.78e-15 |
| Cardinality-critical mode | OSPA | 50/50 | 1.78e-15 |
| Cardinality-critical mode | GOSPA | 50/50 | 1.78e-15 |
| Cardinality-critical mode | Loc. disag. | 50/50 | 1.78e-15 |
| Cardinality-critical mode | Card. disp. | 50/50 | 1.78e-15 |

## Paired proposed-mode tests versus literature adaptations

| Proposed mode | Adaptation | Metric | Mean paired delta | Improved trials | Exact two-sided p |
|:--------------|:-----------|:-------|------------------:|----------------:|------------------:|
| Balanced mode | Zheng-style subdensity GA-LMB adaptation | OSPA | -1.798 | 50/50 | 1.78e-15 |
| Balanced mode | Zheng-style subdensity GA-LMB adaptation | GOSPA | -1.099 | 50/50 | 1.78e-15 |
| Balanced mode | Zheng-style subdensity GA-LMB adaptation | Loc. disag. | -11.582 | 50/50 | 1.78e-15 |
| Balanced mode | Zheng-style subdensity GA-LMB adaptation | Card. disp. | -0.781 | 50/50 | 1.78e-15 |
| Balanced mode | Gao-style local-trust GA-LMB adaptation | OSPA | -1.553 | 50/50 | 1.78e-15 |
| Balanced mode | Gao-style local-trust GA-LMB adaptation | GOSPA | -2.294 | 50/50 | 1.78e-15 |
| Balanced mode | Gao-style local-trust GA-LMB adaptation | Loc. disag. | -2.526 | 50/50 | 1.78e-15 |
| Balanced mode | Gao-style local-trust GA-LMB adaptation | Card. disp. | -1.250 | 50/50 | 1.78e-15 |
| Cardinality-critical mode | Zheng-style subdensity GA-LMB adaptation | OSPA | -1.900 | 50/50 | 1.78e-15 |
| Cardinality-critical mode | Zheng-style subdensity GA-LMB adaptation | GOSPA | -1.104 | 49/50 | 9.06e-14 |
| Cardinality-critical mode | Zheng-style subdensity GA-LMB adaptation | Loc. disag. | -11.558 | 50/50 | 1.78e-15 |
| Cardinality-critical mode | Zheng-style subdensity GA-LMB adaptation | Card. disp. | -0.906 | 50/50 | 1.78e-15 |
| Cardinality-critical mode | Gao-style local-trust GA-LMB adaptation | OSPA | -1.654 | 50/50 | 1.78e-15 |
| Cardinality-critical mode | Gao-style local-trust GA-LMB adaptation | GOSPA | -2.298 | 50/50 | 1.78e-15 |
| Cardinality-critical mode | Gao-style local-trust GA-LMB adaptation | Loc. disag. | -2.502 | 50/50 | 1.78e-15 |
| Cardinality-critical mode | Gao-style local-trust GA-LMB adaptation | Card. disp. | -1.374 | 50/50 | 1.78e-15 |
