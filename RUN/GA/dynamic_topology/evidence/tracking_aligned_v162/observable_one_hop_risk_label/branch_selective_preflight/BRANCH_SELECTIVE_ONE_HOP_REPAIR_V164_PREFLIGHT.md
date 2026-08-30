# V164 branch-selective one-hop repair preflight

- Preset / seed: `x36-formation-fov / 211`
- Privileged repair pages / times: `[5 7 8] / [76 78 79]`
- Receiver-time cells: `30`
- Response accounting: `conservative full-label bytes for every policy`

| Policy | Full / existence-only | Immediate E-OSPA / RMSE gain | Harmful cells E/R | Projected mean gain E/R | Min formation gain E/R | Byte saving | Gate |
|:--|--:|:--|:--|:--|:--|--:|:--:|
| full-label | 120 / 0 | +639.1638 / +456.4127 | 0 / 13 | +11.821% / +10.561% | +5.915% / -13.960% | +0.295% | 0 |
| existence-only-when-shared | 24 / 96 | +232.2325 / -3680.6011 | 1 / 30 | +10.140% / -13.393% | +5.915% / -228.402% | +0.295% | 0 |
| chi2-95-spatial-else-existence | 44 / 76 | +257.5960 / -3687.3932 | 1 / 30 | +10.245% / -13.433% | +5.915% / -228.970% | +0.295% | 0 |
| chi2-99-spatial-else-existence | 49 / 71 | +274.2330 / -3672.8272 | 1 / 30 | +10.313% / -13.348% | +5.915% / -227.743% | +0.295% | 0 |
| chi2-999-spatial-else-existence | 49 / 71 | +274.2330 / -3672.8272 | 1 / 30 | +10.313% / -13.348% | +5.915% / -227.743% | +0.295% | 0 |

## Formation projections

- `full-label` E-OSPA: `[13.818 7.7598 17.21 8.9697 18.477 5.9151]%`; RMSE: `[25.846 5.7725 14.275 1.789 -13.96 23.294]%`
- `existence-only-when-shared` E-OSPA: `[13.818 7.7598 9.2014 8.9697 15.808 5.9151]%`; RMSE: `[25.846 5.7725 -228.4 1.789 -226.96 23.294]%`
- `chi2-95-spatial-else-existence` E-OSPA: `[13.818 7.7598 9.8669 8.9697 15.81 5.9151]%`; RMSE: `[25.846 5.7725 -228.97 1.789 -226.97 23.294]%`
- `chi2-99-spatial-else-existence` E-OSPA: `[13.818 7.7598 10.305 8.9697 15.81 5.9151]%`; RMSE: `[25.846 5.7725 -227.74 1.789 -226.97 23.294]%`
- `chi2-999-spatial-else-existence` E-OSPA: `[13.818 7.7598 10.305 8.9697 15.81 5.9151]%`; RMSE: `[25.846 5.7725 -227.74 1.789 -226.97 23.294]%`

## Evidence boundary

V164 is an opened, nonrecursive Bernoulli-branch mechanism screen. It reuses the privileged F3/F5 V163 cell schedule and the frozen V162 minimum-risk source and positive-risk Top-4 label selector. Policies use only current receiver/source Bernoulli GM densities. For shared labels they either copy the complete density or copy only existence evidence while retaining receiver spatial content; the chi-square rules gate a complete spatial copy by present-time two-dimensional Mahalanobis compatibility. Missing receiver labels always require a complete source density. Truth scores immediate E-OSPA and matched-position RMSE only. Projected recursive metrics add snapshot deltas to V162 and conservatively charge every policy the original full-response V162 bytes. Passing authorizes an actual recursive branch-selective probe, not an online or validation claim.
