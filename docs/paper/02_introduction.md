# Introduction

## Paper-Ready Introduction Draft

Distributed multi-sensor multi-object tracking is a core capability in surveillance, autonomous systems, and cooperative perception. In many practical sensor networks, however, each node has only partial spatial coverage, local measurements have unequal quality, and internode communication is constrained by bandwidth limitations, packet drops, or temporary outages. Under these conditions, the fusion stage must improve global situational awareness without assuming that cross-correlations among local posteriors are known. Labeled random finite set methods provide a principled Bayesian framework for this problem by jointly representing target number, kinematic states, and identities in a single multi-object posterior \cite{Vo2014LRFS,Reuter2014LMB}.

Within the labeled-RFS family, the generalized labeled multi-Bernoulli and labeled multi-Bernoulli filters provide an effective trade-off between tracking accuracy and computational tractability, and they have become standard references for multi-object posterior propagation and track extraction \cite{Vo2014LRFS,Reuter2014LMB,Vo2019MSGLMB}. For multi-sensor fusion, Kullback-Leibler average (KLA), equivalently geometric-average or generalized covariance intersection fusion, is particularly attractive because it provides a conservative rule for combining posteriors when unknown cross-correlations prevent exact Bayesian fusion \cite{Battistelli2014KLA,Hlinka2014ICI}. This idea has also been extended to labeled multi-object densities in distributed settings, where robust formulations have been proposed to mitigate the sensitivity of labeled fusion to inconsistent local labels \cite{Li2018RobustDistributedLRFS}.

Despite these advances, choosing fusion weights remains a practical bottleneck. Many distributed KLA-style implementations still rely on Fixed Metropolis, uniform weights, or topology-derived rules such as Metropolis weighting. Such choices are often too rigid for communication-constrained sensor networks, because the informativeness of a local posterior and the reliability of the communication path that delivers it can both vary significantly over time. Prior work in centralized multi-view LMB fusion has shown that adaptive information-aware weighting can outperform constant GCI weights when sensors have different fields of view \cite{Wang2018CentralizedLMBFusion,Gostar2021CentralizedCooperativeLMB}. Consensus-based distributed LMB tracking has also explored automatic track-level weight evolution for sensors with different fields of view \cite{Shen2022ConsensusLMB}. More recently, distributed heterogeneous labeled-RFS fusion has been studied from an information-geometric perspective \cite{CaoZhao2025InfoGeometryFusion}. A parallel arithmetic-average density-fusion line has also proposed Fisher-information-proportional weights for asynchronous multi-rate filters after continuous-time time alignment \cite{Li2026FIMultirateAADensityFusion}. However, these formulations still do not directly address the peer-to-peer GA-LMB fusion setting considered here, where the effective contribution of a node depends jointly on posterior content and on how reliably that information is realized at neighboring nodes under heterogeneous packet loss.

A second limitation is that posterior covariance alone does not fully characterize whether a local posterior makes a reliable existence decision. In GA-LMB fusion, a node may report a relatively precise state estimate for a Bernoulli component while still being ambiguous about whether the corresponding object truly exists. Conversely, communication statistics can indicate which packets were delivered, but they do not describe how decisive the received posterior is about target existence. This observation suggests that an adaptive fusion rule for communication-constrained LMB fusion should combine at least three complementary cues: state precision, realized link reliability, and existence decisiveness.

Motivated by this gap, this paper proposes a communication-aware adaptive fusion-weight allocation framework for distributed GA-LMB fusion. The design factorizes each fusion weight into covariance-quality, realized link-quality, and existence-confidence terms, then applies a branch-aware refinement that decouples the spatial and existence-weight paths with weak structure-aware modulation. The retained Balanced configuration uses the resulting instantaneous normalized weights without EMA smoothing or final-weight floors. The Cardinality-critical mode preserves this backbone and adds FID-FIA only to the existence branch.

The method is evaluated in an eight-sensor distributed formation-tracking scenario composed of two four-sensor formations under a tiered heterogeneous packet-loss model. The controlled 50-trial ablation shows that realized link quality supplies the dominant gain, covariance provides an additional cardinality benefit, and the combined branch-aware refinement gives a smaller but repeatable spatial-consensus improvement. Existence confidence has little isolated increment on the covariance-link backbone. The Cardinality-critical mode further reduces cardinality dispersion and local cardinality error, while increasing localization disagreement and local RMSE. EMA/floor stabilization is rejected because its broad degradation outweighs its small local-RMSE benefit.

The main contributions of this paper are as follows.

1. We propose a communication-aware adaptive fusion-weight allocation family for distributed KLA-based GA-LMB fusion under communication constraints.
2. We introduce a factorized quality model that combines posterior covariance, realized link quality, and existence confidence, together with decoupled spatial and existence paths and weak structure-aware modulation.
3. We provide a detailed 50-trial component ablation with disagreement, local tracking, and runtime metrics, explicitly separating the dominant link-quality effect from smaller refinements and a negative EMA/floor diagnostic.

## Wording Rule

- Replace every bare `4+4` mention in the introduction with `an eight-sensor distributed formation scenario composed of two four-sensor formations`.
- If a shorter expression is needed after first mention, use `the dual-formation eight-sensor scenario`.

## Citation Keys Used Here

- `Vo2014LRFS`
- `Reuter2014LMB`
- `Vo2019MSGLMB`
- `Battistelli2014KLA`
- `Hlinka2014ICI`
- `Li2018RobustDistributedLRFS`
- `Wang2018CentralizedLMBFusion`
- `Gostar2021CentralizedCooperativeLMB`
- `Shen2022ConsensusLMB`
- `CaoZhao2025InfoGeometryFusion`
- `Li2026FIMultirateAADensityFusion`

- Full reference details and links are collected in `10_reference_seed.md`.
