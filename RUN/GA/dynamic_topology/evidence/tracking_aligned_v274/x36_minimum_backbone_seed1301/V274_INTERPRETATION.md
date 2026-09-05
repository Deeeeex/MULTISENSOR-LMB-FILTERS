# X36: sparse causal routing retains a cost advantage, with a set-estimation gap

The frozen three-arm, 160-step experiment completed at source `c9c589d`.
The minimum causal backbone improves conditional set RMSE by 47.655% and
reduces attempted posterior bytes by 21.830% relative to the fixed formation
tree. Its E-OSPA gain is only 0.368%, however, and focus-window consistency
worsens by 0.776%. This is a useful cross-scale development result, but it
does not meet the joint tracking, communication and consistency objective.

The stored result also exposes a cardinality tradeoff:

| Arm | Mean absolute cardinality error | Focus cardinality error | Full consistency | Terminal consistency |
|:--|--:|--:|--:|--:|
| Fixed formation tree | 18.455729 | 18.085259 | 141.128389 | 142.957584 |
| Full causal repair | 18.463194 | 18.097635 | 140.010280 | 136.270040 |
| Minimum causal backbone | 18.597396 | 18.242024 | 141.581990 | 134.961622 |

All columns are lower-is-better. Cardinality error rises by 0.141667 for the
sparse arm versus fixed routing. Conditional RMSE is computed over matched
states, so its large decrease alone cannot establish that more targets are
tracked correctly. Neither the cardinality increase nor the consistency loss
can be hidden by reporting terminal consistency, which improves by 5.593%.

Full causal repair provides evidence that dynamic repair can improve E-OSPA,
conditional RMSE and consistency in the executed X36 case, but increases
attempted bytes by 5.708%. Comparing full and sparse repair isolates the
cost-quality tradeoff induced by the retained input set. It does not prove
which individual omitted edge caused the sparse arm's cardinality error.

The method foundation remains causal tree repair plus sparse formation
backbones. The next decision should address actual received inputs and
finite-round set estimation, with one controlled change at a time. Faster
mixing, planned strong connectivity and planned matrix balance are not
surrogates for this empirical objective.

Evidence: `CROSS_SCALE_MINIMUM_BACKBONE_V274_FULL_EPISODE.mat`, fields
`fixedTree`, `fullCausal`, `minimumBackbone`; each contains
`meanAbsoluteCardinalityError`, `focusAbsoluteCardinalityError`,
`meanInterFormationPositionOspa` and `terminalInterFormationPositionOspa`.
This interpretation is a self-check of one opened seed, not independent
validation or a claim of statistical significance.
