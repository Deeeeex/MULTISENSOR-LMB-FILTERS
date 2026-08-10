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

This literature does not certify the V90 rule as exact KLA. In particular,
Gao et al. derive the different-FoV subspace construction under minimum
information loss (an arithmetic opinion pool), whereas this repository uses
KLA/GCI (a geometric opinion pool). Their result supports making the FoV
partition explicit; it does not justify silently transplanting the resulting
fusion formula. V90 therefore tests a censored-existence approximation and
keeps a future exact set-density derivation as a separate requirement.

## Method decision

Route tuning pauses at V89. The already running paired episode remains useful development evidence, but it cannot authorize a generic KLA claim or multistyle expansion by itself. V90 will make the fusion semantics explicit before another topology redesign.

Three receiver modes are required:

1. **Legacy support-renormalized control.** Preserve the current behavior only as an engineering ablation.
2. **Strict common-label KLA.** A missing label at any positively weighted input is treated as zero existence. This is the sparse-representation control for the common-label-space formula.
3. **FoV-aware censored-existence fusion.** A missing label is excluded only when current sensor geometry makes that absence uninformative. If the current label density lies in the source's observable region, absence is treated as censored low-existence evidence: its existence upper bound is the transmitted-payload threshold, while it contributes no invented spatial density. The partition uses current sensor pose, range, FoV, expected detection probability and the current predicted label density; it may not use truth, future measurements or tracking outcomes.

The third mode needs a derivation and a clearly named approximation boundary. It must not be described as exact LMB-KLA unless the resulting set-density rule is proved.

## Mathematical contract for the FoV-aware mode

For label \(\ell\), let \(P_\ell\) be the positively weighted inputs that transmit the label. For every missing input \(i\), the receiver evaluates current observability

\[
O_i^\ell = \mathbb{E}_{p^\ell(x)}[p_D^i(x,t)]
\]

from the currently transmitted label density, the current sensor pose and the current FoV model. No target truth is involved. The implementation reuses the registered positive-weight position cubature rule in `computeLmbLabelObservationOpportunity`, so the estimate includes the label covariance crossing the range or angular FoV boundary instead of evaluating only GM component means. Results from multiple present sources are then averaged with their normalized spatial weights. A missing input with negligible FoV mass or low \(O_i^\ell\) is non-informative and is excluded for that label. A missing input with sufficient FoV mass and \(O_i^\ell\) is informative. Since a heavy payload transmits labels only above threshold \(\tau\), its absence provides the censored statement \(r_i^\ell \leq \tau\), not \(r_i^\ell=0\).

The implemented existence update therefore inserts \(\tau\) for informative missing inputs and normalizes existence weights over transmitted plus informative-missing inputs. Spatial KLA uses only transmitted conditional densities; no spatial density is invented for a missing label. With fixed spatial-overlap term, the Bernoulli KLA log odds are monotone in every local log odds, so substituting \(\tau\) yields an upper bound on fused existence over the unknown interval \([0,\tau]\). This bound is deliberately less optimistic than legacy exclusion and less destructive than a zero-existence veto.

Because the missing spatial density is marginalized as neutral rather than reconstructed, this is currently an FoV-aware censored-existence approximation, not an exact set-density KLA theorem. The experiment must retain that name and boundary until a full derivation is completed.

## Frozen experiment decision

After the current V89 run finishes, first perform the committed acquire--broadcast--recovery decomposition. Then compare the physical-tree reference and the frozen V89 route under the three receiver modes on the same M24/X36 scene, measurements, delivery draws and filter seeds.

Interpretation is fixed in advance:

- If V89 improves only under legacy support renormalization, the apparent routing gain is receiver-specific and the generic dynamic-KLA story is rejected.
- If the strict control collapses both reference and candidate, the current pruning/birth representation is incompatible with common-label KLA and must be corrected before route design.
- If FoV-aware fusion preserves a cross-scale V89 gain while the strict and legacy controls explain their respective failure modes, the paper can make a defensible joint contribution: explicit limited-FoV fusion semantics plus causal dynamic routing.
- Multistyle tracking remains closed until the same fusion mode and frozen route pass both M24 and X36. Convoy and relay remain the first generalization scenes; merge-split and curved corridor follow; crossing remains stress-only.

The 5% tracking target is unchanged. This gate changes what is being estimated, not the acceptance threshold.
