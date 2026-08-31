# V200 F3 label-set failure finding

## What the V199 tail failure actually is

The X36 V199 formation-3 RMSE deficit is not distributed across the H=8
window. It is dominated by three receiver-time cells:

| Time | Receiver | Candidate minus static RMSE | Responsible MAP label | Matched error |
|--:|--:|--:|:--|--:|
| 76 | 15 | +138.928 | `[19,13]` | 659.253 |
| 78 | 15 | +146.365 | `[19,13]` | 707.884 |
| 79 | 16 | +154.572 | `[19,13]` | 732.410 |

The captured fused posteriors reproduce the recorded per-cell RMSE exactly.
In all three cells, `[19,13]` has one Gaussian component. Its existence,
weight, mean, and covariance are identical to the static posterior. The
static arm avoids the large matched error only because `[19,13]` is not in
its extracted MAP label set.

The MAP-set differences explain how it enters the candidate output:

| Time / receiver | Candidate / static MAP cardinality | Candidate-only MAP labels | Static-only MAP labels |
|:--|:--:|:--|:--|
| 76 / 15 | 20 / 20 | `[19,13]` | `[31,24]` |
| 78 / 15 | 20 / 18 | `[13,9]`, `[7,5]`, `[19,13]` | `[31,24]` |
| 79 / 16 | 20 / 17 | `[7,5]`, `[13,9]`, `[19,13]` | none |

At `t=76/78`, sparse fusion has reduced `[31,24]` from static existence
`0.964/0.493` to `0.256/0.197`, allowing the stale `[19,13]` hypothesis to
take its place. At `t=79`, the static arm emits only 17 states while V199 emits
20; the matched RMSE then assigns the additional stale state to an otherwise
unmatched truth target. This is a label-set/existence-ranking failure with a
localized stale spatial density, not a general degradation of all F3 state
estimates.

## A causal label source is already observable

The current physical-neighbor candidate bank contains joint-positive
complete-label replacements in every affected cell. More importantly, the
existing truth-free `risk_reduction` score ranks `[19,13]` first without using
the numeric label key, time, formation ID, or truth:

| Time / receiver | Max-risk source-label | Top / runner-up observable risk | Immediate E-OSPA gain | Immediate RMSE gain |
|:--|:--|--:|--:|--:|
| 76 / 15 | source 23, `[19,13]` | 0.3269 / 0.1597 | +6.789 | +131.293 |
| 78 / 15 | source 23, `[19,13]` | 0.3204 / 0.1646 | +6.938 | +140.133 |
| 79 / 16 | source 22, `[19,13]` | 0.3171 / 0.1775 | +6.915 | +145.656 |

The exact best source under truth differs slightly, but the observable
max-risk source is already strongly joint-positive and the top score is
roughly twice the runner-up. Therefore the missing mechanism is not a learned
label identity or a formation-specific schedule. It is an online trigger for
a *dominant receiver-level label risk* followed by a bounded complete-label
update.

## Metric implication

The present Hungarian-matched RMSE averages only matched pairs and does not
penalize unmatched truth or estimated states. Dropping a difficult track can
therefore improve RMSE even when the estimated set is not better. This is why
V199 can improve F3 E-OSPA by `+7.711%` while its F3 RMSE is `-29.799%`.

The weakest-formation RMSE gate remains useful as a warning, but it should not
be treated as a standalone correctness criterion. Future reporting should
keep E-OSPA or GOSPA as the set-quality metric and pair RMSE with an explicit
cardinality/readout condition. The method must not improve RMSE merely by
suppressing outputs; a repair is acceptable only when it is also non-harmful
to the set metric.

## Method decision

The interpretable controller now needs three actions:

1. **No-op** when omission remains locally safe.
2. **Full formation restore** when the defect is diffuse across the label set
   or reflects unsupported set entry/cardinality, as in the M24 mechanism.
3. **Receiver-specific complete-label residual KLA** when one source-label
   candidate has a dominant risk reduction and positive spatial-precision
   evidence, as in X36 F2 and the three F3 cells above.

The next recursive experiment should not hard-code `[19,13]`. It should use a
scale-normalized dominance rule over current candidate scores, select the
max-risk source-label, and apply residual label KLA only at the implicated
receiver. The first controlled teacher should cover the three localized F3
cells and retain the already validated F2 action; full-posterior restoration
should remain the conservative fallback for non-localized failures.

## Evidence boundary

V200 is post-hoc attribution on the opened X36 seed-211 trajectory. Truth is
used only to identify the responsible matched estimate and to score immediate
teacher actions. The proposed observable score is truth-free, but its trigger
and residual-KLA execution have not yet been evaluated recursively or on new
seeds. V187 therefore remains the current balanced best and the
main-document current-best snapshot.
