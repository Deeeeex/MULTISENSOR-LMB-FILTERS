# Describe missing-prediction arrivals after the completed causal case

This is a descriptive continuation, motivated by the exposed reliable GCE
result: same-frame-pruned reimports decreased while the established target
failure remained. It introduces no intervention, score substitution, new
threshold, retention horizon, method selection or extension of the failed
known-local-censor rule. Keep that experiment's fixed gate and all outcomes.

After the twelve native trajectories and their complete audits finish,
freeze all twelve source files, the case's final verification and these
analysis scripts before producing this census. Use all three methods,
original/refined, both links, all 240 frames, both robots. Report full and
unchanged GT5/2m/frames53--122 windows, for all labels and the fixed target
neighbourhood. These are dependent exact-label events, not object counts.

Enumerate every retained fusion output whose local input is absent. Divide
it into (1) present in this frame's local prediction and then pruned,
(2) absent from the current prediction and never predicted at this receiver
before, and (3) absent now but predicted earlier. Preserve its exact source
labels. For class 3, find the last local prediction and local update, actual
posterior pruning outcome, any known-censor intervention at that frame,
and the peer's actual retained posterior for the same exact label.

Count a direct one-frame rebound only when the last local prediction was
at t-1, the old fusion proposal would have retained the label, the actual
refinement dropped it, and the peer retained that exact label. At t the
receiver has no predicted label, receives the represented peer label,
again admits a self censor of 0.001, and retains the output. Check each
condition from the native records. This identifies an observed two-frame
path; it does not prove all reimports or the full performance gap use it.
For longer gaps, report every gap length without selecting a horizon.

The producer processes frames forward, maintaining the last native local
prediction and its final state. The independent verifier reconstructs each
event with per-label prediction histories and binary search, separately
rebuilds native final pools, and checks the full event set, all source and
state fields, class totals, gap distributions and direct rebound totals.
Neither program calls the fusion helper or modifies a saved trajectory.

Report whether fewer immediate reimports were accompanied by later
arrivals of labels that the receiver had already lost, and distinguish
these from first arrivals. Keep the original cause question bounded:
this can explain a limitation of the tested intervention; it cannot turn
a failed rule into a GCE-specific method advantage or exclude other causes.
