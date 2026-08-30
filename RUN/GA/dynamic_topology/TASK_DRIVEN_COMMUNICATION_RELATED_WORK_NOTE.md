# Task-driven communication: related-work positioning

## Why this literature matters

The emerging collaborative-perception literature does not try to reproduce
every transmitted tensor exactly.  It learns when, with whom or where to
communicate so that the downstream perception task is preserved under a
bandwidth budget.  That principle supports the present shift from generic
posterior compression toward decision-aware LMB communication, but the object
being protected here is different: a labeled multi-object posterior and its
set-valued extraction decision, not a neural feature map.

## Closest methodological analogues

| Work | Communication decision | Useful connection | Difference from this work |
|:--|:--|:--|:--|
| Liu et al., *When2com: Multi-Agent Perception via Communication Graph Grouping*, CVPR 2020 | Learns communication groups and when agents should communicate | Establishes learned, state-dependent communication graphs for bandwidth-efficient multi-agent perception | End-to-end visual perception; no labeled-RFS posterior, KLA semantics, explicit set-cardinality certificate or deterministic graph fallback |
| Hu et al., *Where2comm: Communication-Efficient Collaborative Perception via Spatial Confidence Maps*, NeurIPS 2022, DOI `10.52202/068431-0352` | Sends spatially sparse regions selected by confidence maps and adapts them to the bandwidth budget | Closest high-level analogy to transmitting only task-critical information | Selects spatial neural features; the present method selects omission, full-posterior release or label-wise KLA using LMB extraction risk and exact payload bytes |
| Yang et al., *How2comm: Communication-Efficient and Collaboration-Pragmatic Multi-Agent Perception*, NeurIPS 2023, DOI `10.52202/075280-1093` | Uses mutual-information-aware spatial/channel sparsification and delay compensation | Shows that information value, latency and heterogeneous collaboration should be modeled jointly | Requires learned features and transformers; the present safety layer is analytic and works directly on existence probabilities and Gaussian-mixture label densities |
| Utkovski et al., *Semantic Communication for Edge Intelligence: Theoretical Foundations and Implications on Protocols*, IEEE IoT Magazine 2023, DOI `10.1109/IOTM.001.2300167` | Frames communication resource use around the downstream task rather than bit-perfect reconstruction | Provides the broader task-oriented communication interpretation | The present work supplies a concrete set-estimation task object: the LMB cardinality PMF, MAP label set, KLA update and tracking/consensus metrics |

Primary pages used for verification:

- CVPR 2020: <https://openaccess.thecvf.com/content_CVPR_2020/html/Liu_When2com_Multi-Agent_Perception_via_Communication_Graph_Grouping_CVPR_2020_paper.html>
- NeurIPS 2022: <https://proceedings.neurips.cc/paper_files/paper/2022/hash/1f5c5cd01b864d53cc5fa0a3472e152e-Abstract-Conference.html>
- NeurIPS 2023: <https://proceedings.neurips.cc/paper_files/paper/2023/hash/4f31327e046913c7238d5b671f5d820e-Abstract.html>

## Paper-facing distinction

The defensible novelty is not merely "use a GNN to choose edges."  A stronger
story is to formulate posterior communication as a task-oriented action with
an analytic safety certificate:

1. V99 proposes low-bandwidth omission from current posterior and topology
   information.
2. A set-level certificate compares the full and omitted LMB cardinality
   distributions and MAP label boundary using current existence summaries.
3. If omission can change an unsupported extraction decision, the affected
   receiver or formation falls back to the ordinary full posterior.
4. If the risk is localized to one supported label, a mixture-aware label-wise
   KLA action supplies the smaller correction.
5. A learned model or GNN may rank only the actions that survive the analytic
   feasibility and safety projection.

For an LMB with existence vector `r`, the cardinality PMF is a
Poisson-binomial distribution.  Let the full-posterior reference PMF have a
unique MAP cardinality with probability margin `Delta`, and let the compressed
candidate PMF differ from it by at most `delta` in the infinity norm.  The MAP
cardinality is guaranteed unchanged whenever `delta < Delta / 2`.  An
analogous boundary condition on sorted existence probabilities preserves the
top-`n` MAP label set.  These simple certificates give the work a concrete
estimation-theoretic contribution that learned collaborative-perception
methods do not provide.

This note is a positioning aid.  The cited task-oriented methods do not prove
that the proposed LMB certificate or release action works; that evidence must
come from the paired M24/X36 recursive experiments.
