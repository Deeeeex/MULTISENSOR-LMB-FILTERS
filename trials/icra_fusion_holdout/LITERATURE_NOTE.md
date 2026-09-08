# Evidence-ceiling contribution boundary

Checked the publisher's full article and Crossref metadata on 2026-09-08:
[Sun et al., IET Radar, Sonar & Navigation 2019](https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/iet-rsn.2018.5293).
The paper uses neighboring-cell amplitude likelihood information in LMB
tracking to distinguish marine targets from clutter. Its introduction also
describes earlier amplitude-likelihood-ratio association methods. Thus adding
an observation mark or likelihood ratio to the local LMB update is established
practice, and is an information-matched control here, not a novelty claim.
The publisher and Crossref agree on all four authors, volume 13, issue 6,
pages 983--991 and DOI. The raw metadata and normalized BibTeX are adjacent.

The candidate contribution concerns how current direct-association support
limits positive changes caused by recency weights in the Bernoulli fusion
step. The scalar constrained-KL expression is an interpretation of that
specific rule. It does not establish a calibrated statistical upper bound,
consistency of cardinality, recovery of independent likelihood evidence,
unknown-correlation Bayes optimality or convergence of consensus.

The existing paper's primary-source boundaries for LMB, KLA, information
weighting, separate cardinality/localization fusion, FoV label partitioning
and trajectory consensus remain applicable. The final paper must compare
the candidate against baselines given the same score information. An
improvement over unmarked baselines alone would combine observation-model
and fusion-rule contributions. No bibliographic search can prove that a
specific clipping formula has never appeared elsewhere; avoid priority claims.

## Closely related mechanisms checked during the frozen run

| Established mechanism and primary source | Consequence for the candidate claim |
| --- | --- |
| [Uney et al., TAES 2019, DOI 10.1109/TAES.2019.2893083](https://www.pure.ed.ac.uk/ws/files/80294645/consistentRFSfusion_article_muney_final.pdf) show that pointwise consistency of finite-set exponential mixtures does not imply cardinality consistency, and decouple cardinality and localization objectives. | Separate treatment of existence and space is established. ECR retains the spatial-overlap term and adds an input-dependent scalar cap; its constrained surrogate does not inherit their cardinality guarantee. |
| [Yi and Chai, TSP 2021, arXiv:2106.08088v1](https://arxiv.org/abs/2106.08088v1) address spatially varying information confidence by factoring RFS densities into smaller components and fusing them with heterogeneous weights. | Component-specific confidence and nonuniform information weighting are established. The present scope is the current-direct-support constraint on the extra effect of an age reweighting, with spatial weights held fixed for a given input. |
| [Gao, Battistelli and Chisci, Information Fusion 2023, DOI 10.1016/j.inffus.2023.101965](https://www.sciencedirect.com/science/article/pii/S1566253523002816) use a trusted local density to test modified neighbor Bernoulli components and attenuate their influence as disagreement grows. | Neighbor reliability checks and soft confidence reduction are established. ECR uses the ordinary fused existence r0 as its cap reference and current local association support as evidence; it has no attack detector or trusted-receiver guarantee. |

These distinctions follow from the stated objectives and the code-derived
ECR rule; they do not establish an exhaustive novelty result. The quantitative
question remains whether the extra constraint improves the complete paired
same-information comparisons, including missed and false-target costs.

## Additional real-data validation availability

[The official V2X-Seq-SPD quickstart](https://github.com/AIR-THU/DAIR-V2X/blob/main/docs/get_started_spd.md)
provides a vehicle/infrastructure tracking workflow using separate ImvoxelNet
checkpoints and AB3DMOT. The inspected quickstart requires generating detector
outputs through its mmdetection3d/GPU workflow; it does not supply a ready-made
per-frame detection archive in that workflow. This is a possible independent
follow-up dataset, not a completed validation or an available result in the
current experiment. No additional tracking cohort was run in this check.
