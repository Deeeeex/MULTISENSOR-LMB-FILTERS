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
| Hu et al., *Learning Multi-Agent Communication from Graph Modeling Perspective* (CommFormer), 2024 | Jointly optimizes a sparse communication graph and a cooperative policy through continuous graph relaxation | Shows that graph structure can be learned and can transfer across agent counts | Generic centralized-training/decentralized-execution tasks with a fixed sparsity fraction; no physical-link projection, posterior action semantics, exact payload ledger, or multi-object tracking metric |
| Pesce and Montana, *Learning Multi-Agent Coordination through Connectivity-Driven Communication*, *Machine Learning*, 2023, DOI `10.1007/s10994-022-06286-6` | Infers a state-dependent graph and uses graph diffusion to control information flow | Motivates multi-round graph context rather than independent edge scores | Learns latent messages for cooperative navigation; it does not choose among Bayesian fusion operators or protect RFS support and set-estimation tails |
| Ren et al., *Collaborative Multi-Target-Tracking via Graph-Based Deep Reinforcement Learning in UAV Swarm Networks*, *Ad Hoc Networks*, 2025, DOI `10.1016/j.adhoc.2025.103801` | Uses a GNN and multi-agent reinforcement learning for UAV target assignment and swarm-network topology control | The closest direct precedent for data-driven topology control in collaborative multi-target tracking | Optimizes tracking success and fairness through UAV assignment/control; it does not operate on distributed LMB posteriors, distinguish full release from supported-label KLA, or report E-OSPA/RMSE/consensus under exact posterior-byte accounting |
| Lian et al., *Sensor Selection for Decentralized Large-Scale Multi-Target Tracking Network*, *Sensors*, 2018, DOI `10.3390/s18124115` | Selects sensors in a decentralized labeled-RFS tracker using KLA fusion | Establishes task-driven sensor selection inside a labeled multi-object fusion system | Selects sensing nodes rather than posterior communication operators and does not learn a variable-size formation-graph policy |
| Shen et al., *Consensus-Based Labeled Multi-Bernoulli Filter With Event-Triggered Communication*, *IEEE TSP*, 2022, DOI `10.1109/TSP.2022.3154227` | Triggers LMB communication only when a hand-designed condition is met | The most direct event-triggered LMB communication baseline | Binary trigger logic does not distinguish withholding, ordinary full-posterior release and complete-label residual KLA, nor predict their finite-horizon tail-safe values |
| Hu et al., *Where2comm: Communication-Efficient Collaborative Perception via Spatial Confidence Maps*, NeurIPS 2022, DOI `10.52202/068431-0352` | Sends spatially sparse regions selected by confidence maps and adapts them to the bandwidth budget | Closest high-level analogy to transmitting only task-critical information | Selects spatial neural features; the present method selects omission, full-posterior release or label-wise KLA using LMB extraction risk and exact payload bytes |
| Yang et al., *How2comm: Communication-Efficient and Collaboration-Pragmatic Multi-Agent Perception*, NeurIPS 2023, DOI `10.52202/075280-1093` | Uses mutual-information-aware spatial/channel sparsification and delay compensation | Shows that information value, latency and heterogeneous collaboration should be modeled jointly | Requires learned features and transformers; the present safety layer is analytic and works directly on existence probabilities and Gaussian-mixture label densities |
| Utkovski et al., *Semantic Communication for Edge Intelligence: Theoretical Foundations and Implications on Protocols*, IEEE IoT Magazine 2023, DOI `10.1109/IOTM.001.2300167` | Frames communication resource use around the downstream task rather than bit-perfect reconstruction | Provides the broader task-oriented communication interpretation | The present work supplies a concrete set-estimation task object: the LMB cardinality PMF, MAP label set, KLA update and tracking/consensus metrics |

Primary pages used for verification:

- CVPR 2020: <https://openaccess.thecvf.com/content_CVPR_2020/html/Liu_When2com_Multi-Agent_Perception_via_Communication_Graph_Grouping_CVPR_2020_paper.html>
- CommFormer: <https://arxiv.org/abs/2405.08550>
- Connectivity-driven communication: <https://doi.org/10.1007/s10994-022-06286-6>
- Graph-based UAV multi-target tracking: <https://doi.org/10.1016/j.adhoc.2025.103801>
- Decentralized labeled-RFS sensor selection: <https://doi.org/10.3390/s18124115>
- Event-triggered consensus LMB: <https://doi.org/10.1109/TSP.2022.3154227>
- NeurIPS 2022: <https://proceedings.neurips.cc/paper_files/paper/2022/hash/1f5c5cd01b864d53cc5fa0a3472e152e-Abstract-Conference.html>
- NeurIPS 2023: <https://proceedings.neurips.cc/paper_files/paper/2023/hash/4f31327e046913c7238d5b671f5d820e-Abstract.html>

## Paper-facing distinction

The defensible novelty is not "use a GNN to choose edges": learned graph
communication and even GNN-based topology control for collaborative
multi-target tracking now have direct precedents.  The stronger and narrower
story is to formulate **posterior communication as a structured Bayesian
fusion action**:

1. the physical graph and the strong registered route define what information
   can actually flow;
2. the action bank distinguishes no-op/withholding, ordinary full-posterior
   release, and complete-label residual KLA instead of treating every edge as
   one interchangeable neural message;
3. a variable-size formation graph predicts separate finite-horizon values for
   E-OSPA, RMSE, consensus, bytes, receiver-formation performance and affected
   sensor tails;
4. an analytic projector enforces physical connectivity, byte credit,
   cooldown and the support rule that label KLA cannot repair a label absent
   from every receiver;
5. the learned scorer only ranks actions that survive this projector, and
   defaults to no-op when conservative values are non-positive or out of
   distribution.

For an LMB with existence vector `r`, the cardinality PMF is a
Poisson-binomial distribution.  Let the full-posterior reference PMF have a
unique MAP cardinality with probability margin `Delta`, and let the compressed
candidate PMF differ from it by at most `delta` in the infinity norm.  The MAP
cardinality is guaranteed unchanged whenever `delta < Delta / 2`.  An
analogous boundary condition on sorted existence probabilities preserves the
top-`n` MAP label set.  These simple certificates give the work a concrete
estimation-theoretic contribution that learned collaborative-perception
methods do not provide.

The LMB cardinality and label-set stability certificates remain useful
analytic ingredients, but they are not by themselves the paper contribution:
the current evidence shows that aggregate tracking value can hide a severe
sensor-tail regression.  The paper-level claim therefore requires the full
structured action/value/projector system and paired M24/X36 recursive results.
This note is a positioning aid; none of the cited methods proves that the
proposed release or label-KLA policy works.
