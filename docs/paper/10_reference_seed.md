# Reference Seed For Abstract And Introduction

This file collects real citations, DOI links, and suggested BibTeX keys for the current Abstract and Introduction draft. The keys below are the ones used in `02_introduction.md`.

## Core References

| Key | Reference | Link | Why it is used |
| --- | --- | --- | --- |
| `Vo2014LRFS` | Vo, B. N., Vo, B. T., and Phung, D. (2014). *Labeled Random Finite Sets and the Bayes Multi-Target Tracking Filter*. IEEE Transactions on Signal Processing, 62(24), 6554-6567. | https://doi.org/10.1109/TSP.2014.2364014 | Foundational labeled-RFS / GLMB reference. |
| `Reuter2014LMB` | Reuter, S., Vo, B. T., Vo, B. N., and Dietmayer, K. (2014). *The Labeled Multi-Bernoulli Filter*. IEEE Transactions on Signal Processing, 62(12), 3246-3260. | https://doi.org/10.1109/TSP.2014.2323064 | Foundational LMB reference. |
| `Vo2019MSGLMB` | Vo, B. N., Vo, B. T., and Beard, M. (2019). *Multi-Sensor Multi-Object Tracking With the Generalized Labeled Multi-Bernoulli Filter*. IEEE Transactions on Signal Processing, 67(23), 5952-5967. | https://doi.org/10.1109/TSP.2019.2946023 | Multi-sensor labeled-RFS tracking reference. |
| `Battistelli2014KLA` | Battistelli, G. and Chisci, L. (2014). *Kullback-Leibler Average, Consensus on Probability Densities, and Distributed State Estimation with Guaranteed Stability*. Automatica, 50(3), 707-718. | https://doi.org/10.1016/j.automatica.2013.11.042 | Core KLA / distributed-consensus fusion reference. |
| `Hlinka2014ICI` | Hlinka, O., Sluciak, O., Hlawatsch, F., and Rupp, M. (2014). *Distributed Data Fusion Using Iterative Covariance Intersection*. In IEEE ICASSP 2014, pp. 1880-1884. | https://doi.org/10.1109/ICASSP.2014.6853921 | Iterative CI / distributed fusion reference. |
| `Li2018RobustDistributedLRFS` | Li, S., Yi, W., Hoseinnezhad, R., Battistelli, G., Wang, B., and Kong, L. (2018). *Robust Distributed Fusion With Labeled Random Finite Sets*. IEEE Transactions on Signal Processing, 66(2), 278-293. | https://doi.org/10.1109/TSP.2017.2760286 | Distributed labeled-RFS fusion and label-consistency limitation. |
| `Wang2018CentralizedLMBFusion` | Wang, X., Gostar, A. K., Rathnayake, T., Xu, B., Bab-Hadiashar, A., and Hoseinnezhad, R. (2018). *Centralized Multiple-View Sensor Fusion Using Labeled Multi-Bernoulli Filters*. Signal Processing, 150, 75-84. | https://doi.org/10.1016/j.sigpro.2018.04.010 | Prior adaptive information-aware LMB fusion under limited FoV. |
| `Gostar2021CentralizedCooperativeLMB` | Gostar, A. K., Rathnayake, T., Tennakoon, R. B., Bab-Hadiashar, A., Battistelli, G., Chisci, L., and Hoseinnezhad, R. (2021). *Centralized Cooperative Sensor Fusion for Dynamic Sensor Network With Limited Field-of-View via Labeled Multi-Bernoulli Filter*. IEEE Transactions on Signal Processing, 69, 878-891. | https://doi.org/10.1109/TSP.2020.3048595 | Strong limited-FoV LMB fusion reference for contrast with the present distributed setting. |

## Suggested Citation Map

- Labeled-RFS and LMB foundations: `\cite{Vo2014LRFS,Reuter2014LMB}`
- Multi-sensor labeled-RFS tracking: `\cite{Vo2019MSGLMB}`
- KLA / conservative distributed fusion: `\cite{Battistelli2014KLA,Hlinka2014ICI}`
- Distributed labeled-RFS fusion limitations: `\cite{Li2018RobustDistributedLRFS}`
- Adaptive or limited-FoV LMB fusion for contrast: `\cite{Wang2018CentralizedLMBFusion,Gostar2021CentralizedCooperativeLMB}`

## Writing Guidance

- The abstract should normally contain no citations.
- The introduction should cite the foundations, the KLA fusion line, and the closest adaptive-LMB fusion work.
- The current gap statement should not claim that no one has studied adaptive weights before. The precise claim is narrower: prior adaptive weighting evidence is strongest in centralized or limited-FoV settings, whereas this paper targets distributed GA-LMB fusion under heterogeneous packet loss.
