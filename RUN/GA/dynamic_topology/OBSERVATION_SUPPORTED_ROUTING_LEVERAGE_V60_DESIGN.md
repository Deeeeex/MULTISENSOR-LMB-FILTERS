# V60: observation-supported leverage before label-level routing

## Question

V59 falsified formation retention debt as a sufficient intervention signal:
all 16 safe formation subsets remained below 0.4% strict gain at the three
fresh high-debt convoy states.  One plausible failure mechanism is that the
reference-relative rescue metric values every retained label, even when the
receiver has no current measurement support for that label.

V60 reuses the existing V58 reference caches and partitions each
counterfactual existence rescue using the receiver posterior's current
`detectionAssociationMass`.  A label is positively supported at the frozen
V54 threshold `0.20`; a continuous version weights the rescue by the same
association mass.  Target truth and future outcomes are not inputs.

## Decision rule

The support hypothesis remains useful only if the supported or weighted
signal ranks the previously opened strong radial states above the weak radial
and convoy states while reducing the false-positive strength of V59 times 40,
88 and 128.  If it fails, the research will not add another scalar event gate:
the label-level action search will use direct receiver--sender--label
counterfactual value and retain a conservative reference fallback.

No new tracking outcome, X36 result, model training or validation claim is
opened by this attribution.

## Result

The full 29-state observable scan completes the attribution.  On the ten
states with already-opened H=3 outcomes, Pearson correlation with strict gain
is `0.9134` for raw rescued existence, `0.9476` after requiring current
positive receiver support, and `0.9554` after weighting by association mass.
The correction is causally informative rather than cosmetic: convoy `t=40`
drops from `18.562%` raw rescue to `0.501%` supported rescue, while the strong
radial `t=104` state retains `8.493%` supported and `7.544%` weighted rescue.

Positive measurement support is therefore retained as a necessary
receiver-label feature.  It is not treated as a sufficient scalar trigger:
weak convoy states still overlap in scale with the radial `t=60` state, so
V61 uses direct receiver--sender--label counterfactual actions and preserves
the reference fallback.
