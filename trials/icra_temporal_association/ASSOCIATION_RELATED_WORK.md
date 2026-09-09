# Primary-source context for the association experiments

Checked on 2026-09-09. These entries document the scope stated by the
authors in their abstracts; no published method has been reproduced here.

| Primary source | Relevant scope | Implication for this study |
|---|---|---|
| [Nguyen et al., Distributed Multi-object Tracking under Limited Field of View Sensors](https://arxiv.org/abs/2012.12990) | Label consensus and multi-scan OSPA track association; fusion of track estimates whose outcomes do not update local multi-object densities. | Temporal label consistency is established related work. Our recursive posterior feedback and current-observation summaries need to be described explicitly. |
| [Chen et al., Distributed Multi-Object Tracking Under Limited Field of View Heterogeneous Sensors with Density Clustering](https://arxiv.org/abs/2401.00605) | Density-space clustering for track association and graph-based label consensus to reduce track segmentation. | Global label consistency and segmentation reduction require direct evidence beyond a lower instantaneous wrong-pair rate. |
| [Kropfreiter and Hlawatsch, A Probabilistic Label Association Algorithm for Distributed Labeled Multi-Bernoulli Filtering, FUSION 2020](https://repositum.tuwien.at/handle/20.500.12708/77234) | Marginal association probabilities enter distributed GCI/LMB fusion through a belief-propagation and Gaussian approximation. DOI: 10.23919/FUSION45008.2020.9190440. | Association uncertainty inside density fusion already has a probabilistic formulation. The current deterministic prototype should be specified by its executed decision rule. |
| [Kropfreiter and Hlawatsch, Probabilistic object and label association algorithms for distributed multiobject tracking](https://www.sciencedirect.com/science/article/pii/S1566253526004811) | A soft-association framework for distributed JPDA, JIPDA and LMB filters, with efficient marginalization and Gaussian approximations. DOI: 10.1016/j.inffus.2026.104604. | This directly related extension must be considered when positioning any association-based contribution. The publisher lists volume 137, article 104604, January 2027; the [author institution record](https://repositum.tuwien.at/handle/20.500.12708/230581) records 2026. |

The present frozen experiment tests a narrower mechanism: conditional
current-detection summaries reveal a persistent conflict between tracks
that have inherited the same label. The split arm retains both source
posteriors under distinct labels and lets the next local update continue
them. Whether that mechanism improves tracking is decided by the complete
restored recursive results and matched-fusion controls. No novelty or
comparison-to-these-papers claim follows from this source check.
