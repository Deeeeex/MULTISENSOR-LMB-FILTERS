# V75 replacement innovation-energy finding

The formal Octave calculation reproduces the frozen V75 feasibility pattern.
The maximum conditional replacement innovation energy is `3.298216` for M24
formation 3, `1.096073` for X36 formation 4, and `8.594396` for X36 formation
5.  Against the fixed development scale `5.991465`, the first two are retained
and formation 5 is rejected.

| Exact route generation | Formation | Slot energies | Maximum | V75 spatial classification |
|:--|--:|:--|--:|:--:|
| V71/V72 | M24 f3 | `3.298216, 0.675438` | `3.298216` | safe |
| V73 | M24 f3 | `3.298216, 0.143606` | `3.298216` | safe |
| V71/V72 | X36 f4 | `1.096073, 0.755945` | `1.096073` | safe |
| V73 | X36 f4 | `1.096073, 1.052758` | `1.096073` | safe |
| V71/V72 and V73 | X36 f5 | `8.594396, 0.670267` | `8.594396` | unsafe |

All eight evaluated slots have shared-label coverage.  The candidate delivery
reliabilities are between `0.935` and `0.949`; reliability-weighted values are
reported, but the hard classification uses conditional risk so that a rare
delivery cannot hide a harmful replacement.

This result repairs one specific V74 blind spot.  V73's existence opportunity
and V74's self-posterior Bayes objective both rated X36 formation 5 positively,
while RIE exposes that one of its two sender replacements has a large spatial
innovation relative to the incumbent.  The maximum-slot aggregation makes the
joint formation-4-plus-5 route inherit this risk instead of averaging it away.

The evidence boundary is important.  The conventional two-dimensional
chi-square reference `5.991465` was chosen after inspecting these opened
anchors, and the local LMB posteriors are correlated and can be multimodal.
The number is therefore a development-scale trust-region limit, not a
calibrated p-value or a validation result.  V72 outcomes provide retrospective
formation-level mechanism evidence for the old exact routes.  V73 changed one
slot at each scale and those aligned routes have not been executed, so their
classifications are prospective only.

V75 supports keeping replacement innovation as the direct spatial-conflict
term.  It does not yet provide a complete routing policy or a tracking claim.
The direct layer must still combine this term with reference-supported label
retention and the already-positive affected-formation Bayes objective.  The
separate recovery layer must determine whether a direct-safe pulse can return
toward the reference network over later fusion rounds; whole-network
disagreement is not restored as a zero-tolerance direct gate.
