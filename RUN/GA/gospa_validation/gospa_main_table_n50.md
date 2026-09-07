# Main-scenario GOSPA validation

- Seeds: 2--51 (50 deterministic paired trials)
- OSPA/GOSPA: c=5, p=2; GOSPA alpha=2
- Ground space: complete extracted kinematic state vector
- Core source: `/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS-inffus/RUN/GA/gospa_validation/gospa_core_n50_summary.mat`
- PD source: `/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS-inffus/RUN/GA/gospa_validation/gospa_pd_weighted_n50_summary.mat`

| Arm | OSPA | GOSPA | Loc. disag. | Card. disp. | GOSPA improved | Sign-test p |
|:----|-----:|------:|------------:|------------:|---------------:|------------:|
| Fixed Metropolis | 2.469 +/- 0.220 | 4.780 +/- 0.245 | 2.326 +/- 0.353 | 0.716 +/- 0.224 | 0/50 | 1 |
| PD-weighted GA | 2.177 +/- 0.161 | 4.306 +/- 0.191 | 1.995 +/- 0.233 | 0.588 +/- 0.170 | 50/50 | 1.78e-15 |
| FID-FIA-weighted GA | 1.818 +/- 0.056 | 4.045 +/- 0.107 | 1.643 +/- 0.109 | 0.123 +/- 0.023 | 50/50 | 1.78e-15 |
| Balanced mode | 1.779 +/- 0.073 | 3.909 +/- 0.124 | 1.522 +/- 0.157 | 0.188 +/- 0.031 | 50/50 | 1.78e-15 |
| Cardinality-critical mode | 1.678 +/- 0.050 | 3.904 +/- 0.114 | 1.546 +/- 0.158 | 0.063 +/- 0.017 | 50/50 | 1.78e-15 |
