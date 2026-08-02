# V35 debt-coverage staggered recovery

## Motivation

V30 protects formations 2, 3, and 4 at both t=72 and t=73, then rolling-B3
forces all three cross-formation inputs to return together at t=74. The first
two steps improve tracking and consensus, but the abrupt final return leaves a
1.595% terminal-consensus deficit. V31--v34 show that changing only the t=74
weight or gateway cannot safely fix that endpoint.

V23--v25 do not test the present mechanism. They freeze multi-step actions at
t=72 or append passive reference steps. V35 instead observes the live t=73
posterior and begins recovery one step before the hard connectivity boundary.

## Causal release rule

The v30 retention-debt controller remains the incumbent. For its currently
suspended formations, v35 measures how many consecutive selected steps each
cross-formation input has been absent. After at least one absent step, mature
inputs are ordered from lowest to highest current retention debt.

The controller proposes nested releases in that order, but only while the
remaining suspended formations retain at least 80% of the incumbent positive
debt. A proposal must also:

- pass exact reference-relative label retention and decision-threshold gates;
- pass sensor- and formation-level rolling-B3;
- keep at least one message below the reference payload; and
- reduce one-round expected posterior disagreement by at least 0.25% relative
  to continuing the incumbent suspension.

The first eligible nested release is selected. If none is eligible, v35 keeps
the causal v30 action. This uses at most F release probes on top of v30's
linear-size control and therefore remains scalable in formation count.

## Frozen opened-state test

Only the existing M24 seed-211 trajectory is used. At t=73, v30's debts are
approximately `[1.03%, 5.23%, 1.24%, 4.92%]`, and formations 2, 3, and 4 have
been suspended for one step. The preregistered 80% coverage rule may release
formation 3 while retaining formations 2 and 4. No truth, future measurement,
tracking score, additional M24 state, X36, or X48 is available during this
preflight.

If a clean preflight selects a nonreference staggered route, exactly one paired
H=3 rerun on the already-opened t=72 state is authorized. The unchanged gate
requires at least 2% mean tracking gain, nonnegative formation and worst-sensor
gains, nonnegative window and terminal consensus gains, nonnegative attempted
byte saving, and selected rolling-B3 passage. Failure closes this controller
before broader evidence or GNN training.
