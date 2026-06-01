# Related Work

## Paper-Ready Related-Work Draft

Distributed multi-sensor multi-object tracking has a mature foundation in random finite set (RFS) and labeled-RFS filtering. In particular, the generalized labeled multi-Bernoulli (GLMB) and labeled multi-Bernoulli (LMB) families provide principled Bayesian representations of target number, kinematic state, and label information, and they remain standard references for multi-object posterior propagation, track extraction, and multi-sensor update design \cite{Vo2014LRFS,Reuter2014LMB,Vo2019MSGLMB,Vo2024OverviewLRFS}. These works establish the statistical backbone on which the present paper builds, but they do not by themselves resolve how fusion weights should adapt when local posterior quality and communication reliability differ across sensors.

For distributed fusion under unknown cross-correlations, a second major line of work develops conservative density-combination rules based on Kullback-Leibler averaging, geometric-average fusion, and covariance-intersection-like consensus \cite{Battistelli2014KLA,Hlinka2014ICI}. Within the labeled-RFS setting, this line has been extended to robust distributed fusion of labeled densities, including formulations that explicitly address label mismatching and, more recently, resilience to corrupted peer-to-peer messages \cite{Li2018RobustDistributedLRFS,Gao2023ResilientLMB}. These studies make KLA/GCI-style fusion a natural starting point for distributed LMB tracking, but the weight-selection mechanism is typically fixed, uniform, or driven mainly by topology or consensus protocol considerations rather than by a factorized assessment of posterior quality, realized communication quality, and existence decisiveness.

There is also prior evidence that adaptive weighting can be beneficial when sensors provide heterogeneous information. In centralized multiple-view LMB fusion, information-aware weighting has been used to improve performance under limited and different fields of view \cite{Wang2018CentralizedLMBFusion,Gostar2021CentralizedCooperativeLMB}. More recently, consensus-based distributed LMB tracking has introduced automatic track-level weight evolution for sensors with different fields of view \cite{Shen2022ConsensusLMB}. The closest adaptive-weight precedent for the present work is the distributed heterogeneous labeled-RFS fusion method of Cao and Zhao, which uses information geometry and Fisher-information accumulation, often summarized as an FID-FIA weighting rule, to assign adaptive scalar fusion weights under different sensor architectures and limited fields of view \cite{CaoZhao2025InfoGeometryFusion}. Arithmetic-average density fusion is an important alternative route for heterogeneous unlabeled and labeled RFS filters \cite{Li2024AADensityFusion}, and a recent multi-rate extension of that line derives continuous-time Fisher information and a continuous-time Bayesian Cramer-Rao lower bound, then uses Fisher-information-proportional weights for time-aligned asynchronous filters in both AA and GA fusion \cite{Li2026FIMultirateAADensityFusion}. This reinforces Fisher information as a principled measure of local estimator informativeness, especially when posterior quality changes between sampling instants. The present paper deliberately chooses GA/KLA because it provides conservative fusion under unknown cross-correlations, a logarithmic-opinion-pool structure compatible with distributed consensus, and a clean separation between Gaussian spatial fusion and Bernoulli existence fusion after LMB moment projection.

The present work builds on the insight that Fisher-type information is a useful indicator of heterogeneous sensing informativeness, but it uses that idea differently from both of these precedents. Rather than replacing the whole fusion rule with a single FID-FIA scalar weight, we retain distributed KLA-based GA-LMB fusion, add realized packet-delivery information to the weight backbone, and use branch decoupling to restrict the FID-FIA signal to the Bernoulli existence branch. Unlike multi-rate FI-weighted average fusion, the present study does not attempt to solve asynchronous time alignment or derive continuous-time information bounds; it focuses instead on synchronous distributed GA-LMB fusion under heterogeneous packet delivery. This distinction is central: FID-FIA-weighted GA is empirically strong for cardinality decisions, but a single FID-FIA weight can trade away spatial RMSE. Our method therefore treats FID-FIA as an optional existence/cardinality refinement on top of a communication-aware spatial branch, yielding a Balanced mode for position-sensitive deployment and a Cardinality-critical mode for cardinality-sensitive deployment. These works are important precedents, but they do not directly answer the question studied here: how to allocate adaptive fusion weights for distributed GA-LMB fusion when packet delivery is heterogeneous over time and when the contribution of a node depends jointly on posterior concentration, realized link reliability, target-pair information geometry, and how decisively the posterior supports target existence.

A related but separate literature considers innovation-based consistency assessment and communication-constrained distributed estimation. Innovation statistics such as NIS and NEES are standard tools in tracking and navigation for diagnosing filter consistency, and they continue to be used in recent estimator auto-tuning and consistency-enforcement studies \cite{BarShalom2001Estimation,Chen2023NISAutoTuning}. Likewise, communication constraints such as bandwidth limits, delays, asynchronous updates, and packet losses have long been recognized as central design factors in distributed fusion over sensor networks \cite{Zhang2016CommConstrainedFusion}. The present work uses both ideas in a narrower way. Communication constraints are modeled through realized link-quality terms inside the fusion weights, while innovation consistency is not treated as another monotonic quality reward; instead, it is viewed as an optional penalty because innovation-based scores are structurally coupled with covariance-based posterior-quality terms.

Against this background, the contribution of this paper is deliberately narrower than a broad new fusion framework. We stay within distributed KLA-based GA-LMB fusion, but replace fixed or topology-only weighting with a communication-aware factorized weighting design built from covariance quality, realized link quality, and existence confidence. We then add branch-specific refinements: weak structure-aware modulation for the spatial side and FID-FIA-informed information geometry for the existence side. This positioning differentiates the method from both fixed-weight KLA fusion and prior adaptive formulations that use information-aware scalar weights without explicit branch decoupling under heterogeneous packet loss.

## Positioning Against Prior Work

- Prior labeled-RFS work provides the filtering backbone, but not the current adaptive weight-allocation rule.
- Prior KLA or GCI fusion justifies conservative distributed density fusion, but often with fixed or topology-driven weights.
- Prior adaptive LMB weighting shows that heterogeneous information should not be fused with constant weights; FID-FIA is the closest baseline, but the present method uses its information-geometric signal only in the existence branch rather than as a scalar replacement weight.
- Prior NIS-based work supports using innovation statistics as consistency diagnostics, but the present paper intentionally avoids treating NIS as an independent quality reward.
- The present method therefore targets a specific gap: adaptive weight allocation for distributed GA-LMB fusion under heterogeneous communication quality, with topology retained as a weak spatial refinement and FID-FIA retained as an existence/cardinality refinement.

## Citation Keys Used Here

- `Vo2014LRFS`
- `Reuter2014LMB`
- `Vo2019MSGLMB`
- `Vo2024OverviewLRFS`
- `Battistelli2014KLA`
- `Hlinka2014ICI`
- `Li2018RobustDistributedLRFS`
- `Gao2023ResilientLMB`
- `Wang2018CentralizedLMBFusion`
- `Gostar2021CentralizedCooperativeLMB`
- `Shen2022ConsensusLMB`
- `CaoZhao2025InfoGeometryFusion`
- `Li2024AADensityFusion`
- `Li2026FIMultirateAADensityFusion`
- `BarShalom2001Estimation`
- `Chen2023NISAutoTuning`
- `Zhang2016CommConstrainedFusion`
