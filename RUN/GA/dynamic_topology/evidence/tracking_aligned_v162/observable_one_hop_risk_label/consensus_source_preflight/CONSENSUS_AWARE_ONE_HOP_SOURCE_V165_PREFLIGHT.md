# V165 consensus-aware one-hop source preflight

- Preset / seed: `x36-formation-fov / 211`
- Privileged repair pages / times: `[5 7 8] / [76 78 79]`
- Receiver-time cells: `30`

| Policy | Actions | Immediate E-OSPA / RMSE gain | Harmful cells E/R | Projected mean gain E/R | Min formation gain E/R | Bytes | Saving | Gate |
|:--|--:|:--|:--|:--|:--|--:|--:|:--:|
| minimum-risk-k4 | 120 | +639.1638 / +456.4127 | 0 / 13 | +11.821% / +10.561% | +5.915% / -13.960% | 618368 | +0.295% | 0 |
| receiver-compatible-source-k4 | 120 | +325.3809 / +389.8335 | 0 / 13 | +10.525% / +10.175% | +5.915% / -3.847% | 503120 | +0.698% | 0 |
| neighbor-medoid-source-k4 | 120 | +485.3085 / +468.6774 | 0 / 10 | +11.185% / +10.632% | +5.915% / -5.687% | 521168 | +0.635% | 0 |
| maximum-local-support-source-k4 | 120 | +322.7452 / +386.6528 | 0 / 13 | +10.514% / +10.157% | +5.915% / -3.051% | 478184 | +0.785% | 0 |
| credibility-weighted-risk-k4 | 120 | +335.5778 / +531.2271 | 0 / 4 | +10.567% / +10.994% | +5.915% / -2.427% | 484832 | +0.762% | 0 |
| chi2-95-gated-risk-k4 | 120 | +145.7327 / +520.2934 | 1 / 12 | +9.782% / +10.931% | +5.915% / -8.956% | 529232 | +0.607% | 0 |
| chi2-99-gated-risk-k4 | 120 | +262.3627 / +524.0207 | 0 / 12 | +10.264% / +10.952% | +5.915% / -8.852% | 558200 | +0.505% | 0 |
| credibility-weighted-risk-k1 | 30 | +205.3934 / +413.8989 | 0 / 12 | +10.029% / +10.315% | +5.915% / -2.550% | 311936 | +1.367% | 0 |
| credibility-weighted-risk-k2 | 60 | +216.6169 / +462.0464 | 0 / 8 | +10.075% / +10.593% | +5.915% / -2.550% | 343736 | +1.256% | 0 |
| credibility-weighted-risk-k3 | 90 | +257.4402 / +468.4491 | 0 / 7 | +10.244% / +10.631% | +5.915% / -2.415% | 398912 | +1.063% | 0 |

## Formation projections

- `minimum-risk-k4` E-OSPA: `[13.818 7.7598 17.21 8.9697 18.477 5.9151]%`; RMSE: `[25.846 5.7725 14.275 1.789 -13.96 23.294]%`
- `receiver-compatible-source-k4` E-OSPA: `[13.818 7.7598 13.22 8.9697 14.265 5.9151]%`; RMSE: `[25.846 5.7725 3.6279 1.789 -3.8467 23.294]%`
- `neighbor-medoid-source-k4` E-OSPA: `[13.818 7.7598 17.491 8.9697 14.208 5.9151]%`; RMSE: `[25.846 5.7725 11.19 1.789 -5.687 23.294]%`
- `maximum-local-support-source-k4` E-OSPA: `[13.818 7.7598 13.133 8.9697 14.283 5.9151]%`; RMSE: `[25.846 5.7725 2.9635 1.789 -3.0508 23.294]%`
- `credibility-weighted-risk-k4` E-OSPA: `[13.818 7.7598 12.566 8.9697 15.175 5.9151]%`; RMSE: `[25.846 5.7725 14.839 1.789 -2.4268 23.294]%`
- `chi2-95-gated-risk-k4` E-OSPA: `[13.818 7.7598 9.4394 8.9697 13.329 5.9151]%`; RMSE: `[25.846 5.7725 17.169 1.789 -8.9563 23.294]%`
- `chi2-99-gated-risk-k4` E-OSPA: `[13.818 7.7598 12.509 8.9697 13.331 5.9151]%`; RMSE: `[25.846 5.7725 17.431 1.789 -8.8515 23.294]%`
- `credibility-weighted-risk-k1` E-OSPA: `[13.818 7.7598 10.87 8.9697 13.467 5.9151]%`; RMSE: `[25.846 5.7725 5.0106 1.789 -2.5496 23.294]%`
- `credibility-weighted-risk-k2` E-OSPA: `[13.818 7.7598 11.166 8.9697 13.467 5.9151]%`; RMSE: `[25.846 5.7725 9.0691 1.789 -2.5496 23.294]%`
- `credibility-weighted-risk-k3` E-OSPA: `[13.818 7.7598 12.027 8.9697 13.678 5.9151]%`; RMSE: `[25.846 5.7725 9.5419 1.789 -2.4152 23.294]%`

## Evidence boundary

V165 is an opened, nonrecursive source-ranking screen on the privileged V163 F3/F5 cells. Every ranker enumerates current one-hop complete Bernoulli GM labels and uses only present receiver/source Bayes risk, receiver-source position compatibility, and source agreement across current physical neighbors. Truth evaluates the resulting complete-label Top-4 replacements through immediate E-OSPA and matched-position RMSE; it is not a feature. Projected recursive metrics add these snapshot deltas to V162. Synopsis, request, and complete response bytes are charged conservatively. Passing only authorizes an actual recursive source-policy probe and cannot support an online, validation, or generalization claim.
