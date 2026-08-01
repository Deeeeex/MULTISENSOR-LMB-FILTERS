# Formation H=3 coordinated-subset repair probe v20

## Question

v19 shows that a conservative trust-0.30 action on formations 3 and 4 can
repay about 77.5% of the consensus debt created by the profitable temporal
prefix `[9,13]`, while retaining `+6.416%` mean tracking gain.  However, the
terminal consensus target remains at `-2.589%`.  v20 asks whether the missing
repair is simply coordination scale: three or four formations may need to
change together at the final step.

## Frozen probe

- opened state: `m24-formation-fov / seed 211 / t=72`;
- fixed prefix: `[9,13]`, unchanged from v18 and v19;
- controls: all-reference `[1,1,1]` and prefix-plus-reference `[9,13,1]`;
- third-step repair: the four unordered three-formation subsets followed by
  the all-four-formation subset, all at trust 0.30;
- expanded action indices: `[20,21,22,23,24]`, appended after the exact v19
  local-plus-pair prefix at indices 1--19;
- total arms: seven;
- final decision: all six targets must be nonnegative and mean tracking gain
  must reach at least `3%` for a strong mechanism result;
- all physical, payload, exact-execution, rolling-B3, truth, repair,
  emergency, and infeasibility gates remain unchanged.

The expanded bank must reproduce `[9,13,1]` within `5e-6` percentage points
before any higher-order result is interpreted.  No trust weight, duration,
prefix, state, or feasibility tolerance is tuned after observing the run.

## Stopping rule

A strong strict-feasible action would justify extending a coordinated
sequence teacher to additional unopened M24 development states.  If every
higher-order action remains infeasible, uniform-trust subset enumeration
stops here.  The next method must instead optimize a heterogeneous
multi-formation mode vector against predicted terminal consensus debt.

This is a privileged single-state mechanism probe.  Seeds 223/227, X36, and
final seeds remain unopened and cannot be claimed from this result.

## Result

The expanded bank reproduces `[9,13,1]` within the frozen tolerance, but only
the all-reference arm is strict-feasible.  The strict oracle therefore
remains `[1,1,1]` with `0%` gain, and no strong safe sequence is found.

The best-consensus higher-order sequence is `[9,13,23]`, which applies trust
0.30 to formations 2+3+4 and obtains
`[+6.448, 0, +0.031, -2.395, +1.027, +1.074]%`.  It repays about 79.1% of the
original `-11.486%` consensus debt, only a modest improvement over the 77.5%
repaid by the v19 pair 3+4.  The all-formation action is worse on consensus
at `-2.637%`, despite larger communication savings.

The stopping rule is therefore active: uniform-trust subset enumeration ends
at v20.  Coordination scale is not the missing scalar.  The next method must
choose a heterogeneous mode vector across formations and score its predicted
terminal consensus debt, with the existing tracking, tail, and communication
targets retained as hard safety constraints.
