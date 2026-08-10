# V90 partial-label fusion semantics gate

## Finding

The current distributed receiver is not a generic common-label-space LMB-KLA implementation when input posteriors contain different active label sets. In `fuseLmbPosteriorsByLabel`, a source that does not carry a label is removed from that label's fusion and the remaining weights are renormalized. Consequently, a label carried by only one source passes through that fusion unchanged rather than being suppressed by zero existence at the other positively weighted sources.

This rule entered with commit `1000a9d` as part of the dual-threshold communication prototype. It is a practical response to independently pruned label sets, but it was not derived there as exact LMB-KLA. The current mixture-aware spatial path changes the Gaussian-mixture calculation; it does not change this missing-label rule.

The distinction matters directly to V84--V89. Their route signal is current support that the receiver and incumbent do not possess. Under the current receiver this support can propagate losslessly across a missing-label input. Under common-label-space KLA, all positively weighted Bernoulli components participate and a zero-existence component vetoes the geometric product. Therefore a V89 tracking gain would currently establish a result only for the repository's support-renormalized receiver, not for generic LMB-KLA.

## Literature boundary

Closed-form labeled GCI/KLA assumes consistent label spaces. Work on robust labeled-RFS fusion treats label inconsistency as a separate problem rather than silently dropping missing inputs:

- Li et al., [Robust Distributed Fusion with Labeled Random Finite Sets](https://arxiv.org/abs/1710.00501), show that labeled GCI is sensitive to label inconsistency and propose an explicit robust construction.
- Wang et al., [Distributed Fusion of Labeled Multi-Object Densities Via Label Spaces Matching](https://arxiv.org/abs/1603.08336), match label spaces before GCI fusion.
- Gao et al., [Fusion of labeled RFS densities with minimum information loss](https://arxiv.org/abs/1911.01083), partition exclusive and common fields of view and fuse the resulting label subspaces explicitly.

The repository's current missing-label exclusion resembles an unregistered exclusive-support partition, but absence after pruning, absence inside a sensor's field of view, and absence outside its field of view are not equivalent evidence. They must not share one implicit rule.

## Method decision

Route tuning pauses at V89. The already running paired episode remains useful development evidence, but it cannot authorize a generic KLA claim or multistyle expansion by itself. V90 will make the fusion semantics explicit before another topology redesign.

Three receiver modes are required:

1. **Legacy support-renormalized control.** Preserve the current behavior only as an engineering ablation.
2. **Strict common-label KLA.** A missing label at any positively weighted input is treated as zero existence. This is the sparse-representation control for the common-label-space formula.
3. **FoV-aware partitioned fusion.** A missing label may be excluded only when current sensor geometry makes that absence uninformative. A missing label inside the source's observable region remains negative evidence. The partition must use current sensor pose, range, FoV and the current predicted label density; it may not use truth, future measurements or tracking outcomes.

The third mode needs a derivation and a clearly named approximation boundary. It must not be described as exact LMB-KLA unless the resulting set-density rule is proved.

## Frozen experiment decision

After the current V89 run finishes, first perform the committed acquire--broadcast--recovery decomposition. Then compare the physical-tree reference and the frozen V89 route under the three receiver modes on the same M24/X36 scene, measurements, delivery draws and filter seeds.

Interpretation is fixed in advance:

- If V89 improves only under legacy support renormalization, the apparent routing gain is receiver-specific and the generic dynamic-KLA story is rejected.
- If the strict control collapses both reference and candidate, the current pruning/birth representation is incompatible with common-label KLA and must be corrected before route design.
- If FoV-aware fusion preserves a cross-scale V89 gain while the strict and legacy controls explain their respective failure modes, the paper can make a defensible joint contribution: explicit limited-FoV fusion semantics plus causal dynamic routing.
- Multistyle tracking remains closed until the same fusion mode and frozen route pass both M24 and X36. Convoy and relay remain the first generalization scenes; merge-split and curved corridor follow; crossing remains stress-only.

The 5% tracking target is unchanged. This gate changes what is being estimated, not the acceptance threshold.
