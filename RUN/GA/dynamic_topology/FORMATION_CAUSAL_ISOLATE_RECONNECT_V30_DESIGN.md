# V30 causal retention-debt routing

## Why v29 is insufficient

The best v29 action withholds the registered cross-formation inputs of formations 2, 3, and 4 for one step. It improves tracking by 8.209% and consensus by 23.623% at the intervention step, but the advantage decays after immediately returning to the fixed reference route. At the third step, tracking is 2.525% worse and consensus is 7.995% worse than reference. The three-step mean gain is therefore 1.8145%, below the frozen 2% threshold.

The result establishes useful headroom but does not yet give a deployable rule: the best subset was identified with future tracking outcomes, and a fixed one-step duration is not state adaptive. V30 replaces both choices with an online controller.

## Source-only control signal

For each formation \(f\), the controller compares two one-round KLA predictions constructed from information available now:

1. the registered reference route; and
2. a counterfactual route that withholds only formation \(f\)'s registered cross-formation input.

Let \(\bar N_f^{\mathrm{ref}}\) and \(\bar N_f^{-f}\) denote the formation-average expected cardinalities under these two routes. The retention-debt score is

\[
d_f = \frac{\bar N_f^{-f}-\bar N_f^{\mathrm{ref}}}
           {\max(\bar N_f^{\mathrm{ref}},1)}.
\]

A positive value means that the reference cross input depresses expected existence mass in this one-round counterfactual. This is a truth-free conflict proxy, not a tracking-performance certificate: suppressing an input can also preserve false labels. Every requested action must therefore pass the separate reference-relative label-retention, decision-threshold, expected-cardinality, physical, payload, and rolling-connectivity constraints before execution.

## Hysteretic controller

The controller is recomputed from the current posterior, physical graph, link-delivery probabilities, and the last two selected topologies at every selected step. It never reads target truth, future measurements, or future tracking outcomes.

- A currently active cross input is suspended when \(d_f \geq 2\%\).
- A previously suspended input remains suspended until \(d_f < 1\%\).
- Formations with insufficient reference expected cardinality, unavailable counterfactuals, or unsafe single-formation actions are not requested.
- The requested joint subset is evaluated exactly. If it violates label-retention or rolling-B3 safety, the controller restores formations from lowest to highest debt until the joint action is safe. If none remains, it uses the registered reference route.

The 2%/1% thresholds are frozen development choices obtained from the already-opened M24 state. They require evaluation on additional states and reserved scenarios before they can support a generalization claim.

## Why this is more scalable than subset search

V29 evaluated all \(2^F\) subsets for \(F\) formations. V30 evaluates the reference, \(F\) single-formation counterfactuals, and at most \(F\) joint projections. Its route count is therefore at most \(2F+1\). The expected KLA outcome calculation still scales with the number of receivers and local link-outcome enumeration, but the formation-level action search is linear rather than exponential.

This analytic controller is also a meaningful later target for data-driven approximation: a GNN could estimate the per-formation debt or rank restoration decisions, while the exact safety projection remains outside the learned model. GNN training is not part of v30 and remains sealed until the source-only controller demonstrates multi-state headroom.

## Frozen primary gate

The primary screen compares only the registered reference arm with the controller-selected action on M24, seed 211, times 72--74. A nonreference arm is strong only when all conditions hold:

- mean tracking gain is at least 2%;
- no formation and no worst sensor regresses;
- both window-average and terminal consensus do not regress;
- attempted bytes do not increase;
- every control update is truth-free and future-free; and
- every selected rolling-B3 sensor and formation window passes.

Passing the primary gate may authorize one additional already-opened M24 state. It does not authorize GNN training, X36/X48, or validation. Failure closes v30 without opening more outcome evidence.
