# V100 graph-scale-aware propagation-horizon probe

## Why H=3 is no longer a neutral choice

The H=3 window was inherited from the earlier M24 action screens.  At the
X36 t=72 state, however, the matched static effective graph has a six-formation
directed cycle with diameter five; the full sensor-level directed diameter is
twenty.  A three-step average therefore ends before information can traverse
the formation graph.

V99 provides direct evidence of this mismatch.  Its gain over static is
1.216%, 2.115% and 5.061% at t=72, 73 and 74.  The method first crosses the 5%
threshold at the final observed step, so the 2.802% three-step mean cannot tell
whether the effect is delayed and persistent or merely a one-step peak.

## Registered probe

V100 changes no spatial decision rule.  It compares the matched static arm,
the initial V97 set held fixed, and causal online re-selection for six steps:
the action step plus the formation-level directed diameter of five.  All arms
share the same opened posterior, measurements, link uniforms, filter RNG,
carrier graph, fusion weights and communication model.

This is not post-hoc duration tuning.  H=6 is determined from the observable
registered graph before the longer outcomes are scored.  The result supports a
scale-aware horizon only if the mean gain over all six steps reaches 5%, the
later-step gain does not collapse, every sensor/formation and consensus gate
remains nonnegative, rolling B3 passes, and attempted bytes do not exceed the
static arm.  Failure redirects the method to a more spatially granular action.

## Result

The H=6 online arm lowers mean E-OSPA from 84.581111 for the matched static
arm to 80.807326, a 4.462% gain.  This is stronger than the fixed initial
set's 3.710% but still below the registered 5% gate.  Per-step gains are
1.216%, 2.115%, 5.061%, 6.145%, 4.623% and 7.779%; the post-propagation
minimum is 4.623%, so the effect is neither absent nor stably above 5%.

Worst-sensor, minimum-formation, consensus, B3 and communication gates all
pass, but the gains are spatially uneven.  Formation gains are 2.265%, 2.866%,
5.512%, 8.367%, 7.942% and 0.165%.  Extending the window further is therefore
rejected.  The next method must refine what information crosses an active
gateway or how its protected posterior is disseminated inside a formation;
it must not reuse the previously failed sender-support label heuristic.
