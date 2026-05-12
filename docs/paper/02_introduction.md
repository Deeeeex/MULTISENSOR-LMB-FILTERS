# Introduction

## Paper-Ready Introduction Draft

Distributed multi-sensor multi-object tracking is a core capability in surveillance, autonomous systems, and cooperative perception. In many practical sensor networks, however, each node has only partial spatial coverage, local measurements have unequal quality, and internode communication is constrained by bandwidth limitations, packet drops, or temporary outages. Under these conditions, the fusion stage must improve global situational awareness without assuming that cross-correlations among local posteriors are known. Labeled random finite set methods provide a principled Bayesian framework for this problem by jointly representing target number, kinematic states, and identities in a single multi-object posterior \cite{Vo2014LRFS,Reuter2014LMB}.

Within the labeled-RFS family, the generalized labeled multi-Bernoulli and labeled multi-Bernoulli filters provide an effective trade-off between tracking accuracy and computational tractability, and they have become standard references for multi-object posterior propagation and track extraction \cite{Vo2014LRFS,Reuter2014LMB,Vo2019MSGLMB}. For multi-sensor fusion, Kullback-Leibler average (KLA), equivalently geometric-average or generalized covariance intersection fusion, is particularly attractive because it provides a conservative rule for combining posteriors when unknown cross-correlations prevent exact Bayesian fusion \cite{Battistelli2014KLA,Hlinka2014ICI}. This idea has also been extended to labeled multi-object densities in distributed settings, where robust formulations have been proposed to mitigate the sensitivity of labeled fusion to inconsistent local labels \cite{Li2018RobustDistributedLRFS}.

Despite these advances, choosing fusion weights remains a practical bottleneck. Many distributed KLA-style implementations still rely on fixed weights, uniform weights, or topology-derived rules such as Metropolis weighting. Such choices are often too rigid for communication-constrained sensor networks, because the informativeness of a local posterior and the reliability of the communication path that delivers it can both vary significantly over time. Prior work in centralized multi-view LMB fusion has shown that adaptive information-aware weighting can outperform constant GCI weights when sensors have different fields of view \cite{Wang2018CentralizedLMBFusion,Gostar2021CentralizedCooperativeLMB}. Consensus-based distributed LMB tracking has also explored automatic track-level weight evolution for sensors with different fields of view \cite{Shen2022ConsensusLMB}. More recently, distributed heterogeneous labeled-RFS fusion has been studied from an information-geometric perspective \cite{CaoZhao2025InfoGeometryFusion}. However, these formulations still do not directly address the peer-to-peer GA-LMB fusion setting considered here, where the effective contribution of a node depends jointly on posterior content and on how reliably that information is realized at neighboring nodes under heterogeneous packet loss.

A second limitation is that posterior covariance alone does not fully characterize whether a local posterior makes a reliable existence decision. In GA-LMB fusion, a node may report a relatively precise state estimate for a Bernoulli component while still being ambiguous about whether the corresponding object truly exists. Conversely, communication statistics can indicate which packets were delivered, but they do not describe how decisive the received posterior is about target existence. This observation suggests that an adaptive fusion rule for communication-constrained LMB fusion should combine at least three complementary cues: state precision, realized link reliability, and existence decisiveness.

Motivated by this gap, this paper proposes a communication-aware adaptive fusion-weight allocation method for distributed GA-LMB fusion. The proposed method factorizes each fusion weight into covariance-quality, realized link-quality, and existence-confidence terms, and then uses branch-specific refinements for the two parts of the LMB posterior. A weak structure-aware decoupled KLA refinement regularizes the spatial branch, so graph structure acts only as a mild prior rather than the primary source of weight allocation. An FID-FIA-informed existence refinement then imports the Fisher-information-accumulation idea of Cao and Zhao \cite{CaoZhao2025InfoGeometryFusion}, but restricts it to the Bernoulli existence branch where it improves cardinality decisions without replacing the spatial fusion rule.

The method is evaluated primarily in an eight-sensor distributed formation-tracking scenario composed of two four-sensor formations under a tiered heterogeneous packet-loss model. This setting preserves the historical average communication intensity while introducing persistent cross-node link heterogeneity, making it more suitable for assessing communication-aware weighting than a uniform packet-drop model. A supporting ideal-communication comparison is also included to test whether the same weighting logic remains useful when link degradation is removed. The experimental results show that adaptive weighting substantially improves consensus quality over fixed-weight fusion, that existence confidence provides the missing gain beyond covariance-and-link-only weighting, and that restricting FID-FIA to the existence branch improves cardinality consistency while retaining the spatial-consensus advantages of the communication-aware KLA backbone.

The main contributions of this paper are as follows.

1. We propose a communication-aware adaptive fusion-weight allocation scheme for distributed KLA-based GA-LMB fusion under communication constraints.
2. We introduce a factorized quality model that combines posterior covariance, realized link quality, and existence confidence, together with branch-specific refinements: weak structure-aware modulation for spatial fusion and FID-FIA-informed information geometry for existence fusion.
3. We demonstrate in a tiered packet-loss dual-formation eight-sensor scenario, supported by an ideal-communication comparison, that the proposed design improves consensus performance over fixed-weight and partial-factor baselines.

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

- Full reference details and links are collected in `10_reference_seed.md`.
