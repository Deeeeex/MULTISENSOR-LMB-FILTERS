# Exact counterfactual selective cross-pulse V53

## Method decision

V51 and V52 isolate two complementary facts on X36 convoy seed 1009. V51
withheld only selected cross-formation residual inputs and slightly improved
cardinality and inter-formation consistency, but its raw existence-gap proxy
produced almost no full-horizon tracking gain. V52 evaluated the actual LMB
fusion outcome, but moved the complete residual layer and forced every window
to phase 4; tracking and cardinality then worsened.

V53 combines only the supported parts of those branches:

- retain the V46 dominant layer on every step;
- retain all local within-formation residual messages on the ordinary phase-1
  pulse;
- evaluate only the small set of cross-formation residual inputs with the
  same full LMB fusion and link-delivery model as runtime;
- defer a cross input only when withholding it materially protects the
  receiver formation's expected cardinality and supported labels;
- prohibit the same formation from being deferred on two consecutive pulse
  opportunities and project the selected subset through the previous/current
  pulse union connectivity check.

The action therefore remains V51's selective, communication-saving topology
change. The decision signal changes from a sender/receiver existence-gap
proxy to the actual serve/hold fusion counterfactual that produced useful M24
signals in V30/V35.

## Exact formation debt

At an absolute phase-1 pulse, first evaluate the complete current V46 route.
For each formation with an incoming cross residual edge, evaluate one
candidate that withholds only that formation's cross input and returns its
weight to the affected receiver. Let the formation-average expected
cardinalities under full service and withholding be
`N_full(f)` and `N_hold(f)`. The retention debt is

`d(f) = (N_hold(f) - N_full(f)) / max(N_full(f), 1)`.

A positive debt means that the current cross input suppresses expected
existence mass under the actual runtime fusion model. The existing 2% on
threshold and 1% off threshold are retained from the M24 counterfactual
branch rather than fitted to the opened X36 result.

The requested joint withholding subset is evaluated exactly. It may execute
only if it does not suppress labels supported by the full-service reference,
does not introduce decision-threshold drops, respects the physical graph and
row-stochastic weights, and preserves strong sensor and formation information
flow over the previous/current pulse union. If the joint action fails, restore
formations from lowest to highest debt until it passes; otherwise use V46.

## Scale and learning path

The evaluator requires one full-service route, one single-formation route per
formation, and at most a small number of joint projections. Its action count
is linear in formation count and independent of the number of graph cycles.
For X36 with six formations this is a bounded centralized analytic baseline.

If V53 demonstrates material tracking value on X36 convoy and the two new
scene styles, its per-formation debts and selected actions become teacher
labels for a GNN or other data-driven edge-value model. The learned model
would approximate the expensive counterfactual score; the physical route,
bounded deferral and temporal connectivity projection remain deterministic.

## Experiment decision

The first screen reuses the saved X36 convoy seed-1009 V46 baseline and runs
only V53. It advances only if full-horizon E-OSPA and cardinality both improve
materially without degrading focus tracking. A positive direction is then
tested unchanged on X36 merge-split and curved-corridor before M24. If the
exact selective action remains near-neutral, the evidence no longer supports
topology scheduling alone; the next research direction should move to
label-selective robust fusion rather than another routing heuristic.

The numerical gate is frozen before reading the V53 result:

- full-horizon position E-OSPA improvement must be at least `+1.0%`;
- mean absolute cardinality-error improvement must be at least `+1.0%`;
- focus-window position E-OSPA may not degrade by more than `0.5%`;
- attempted posterior messages may not exceed V46.

This is only a development gate, not evidence of stable generalization. If it
passes, all routing and retention settings are frozen. The stability stage
uses X36 convoy, merge-split and curved-corridor with disjoint seeds and then
the corresponding M24 scenes. A paper-level positive result requires positive
mean full-horizon and cardinality improvements at both scales, wins in at
least two thirds of paired cases, and no scene family with a mean focus-window
degradation larger than `1.0%`. Statistical intervals and the final seed count
are reported over paired scene-seed differences rather than inferred from the
single opened development case.
