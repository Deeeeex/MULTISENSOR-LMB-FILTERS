# Formation H=3 pair-repair sequence probe v19

## Question

v18 shows that the local sequence `[9,13,12]` can raise M24 mean tracking to
`+8.623%`, restore communication, and repay 67.6% of the original consensus
debt, but one-formation-at-a-time repair leaves consensus at `-3.717%`.
v19 asks whether changing two formations together at the final step can close
that remaining gap without sacrificing the accumulated gain.

## Frozen probe

- opened state: `m24-formation-fov / seed 211 / t=72`;
- fixed prefix: `[9,13]`, corresponding to formation 3/trust 0.50 followed by
  formation 4/trust 0.70;
- controls: all-reference `[1,1,1]` and prefix-plus-reference `[9,13,1]`;
- repair actions: all six unordered formation pairs at trust 0.30, combined
  bank indices `[14,15,16,17,18,19]`;
- total arms: eight;
- final decision: original six targets must all be nonnegative and mean
  tracking gain must reach at least `3%` for a strong mechanism result;
- per-step physical, payload, exact-execution, rolling-B3, truth, repair,
  emergency, and infeasibility gates remain unchanged.

The combined bank preserves local action indices 1–13 exactly and appends the
six pair actions.  `[9,13,1]` must reproduce the v18 six-target vector before
the pair results are interpreted.

## Decision boundary

A strong strict-feasible pair repair would establish coordinated temporal
headroom and justify expanding the sequence teacher to more opened M24
states.  Failure would rule out the existing conservative trust-0.30 pair
bank as the missing repair mechanism; the next action must be an explicitly
optimized multi-formation projection, not another duration or scalar-trust
sweep.

This is a privileged single-state probe.  Seeds 223/227, X36, and final seeds
remain unopened.

## Result

Only the all-reference arm is strict-feasible, so the v19 oracle remains
`[1,1,1]` with `0%` gain.  The best communication-safe high-gain pair is
`[9,13,18]`, which obtains
`[+8.656, 0, +0.031, -3.554, +0.238, +0.249]%`.  The pair that most reduces
consensus debt is `[9,13,19]`, with
`[+6.416, 0, -0.001, -2.589, +0.812, +0.849]%`.

Thus the existing trust-0.30 pair bank still cannot close the terminal
consensus gap, but coordinated repair is not inert: `[9,13,19]` repays about
77.5% of the original `-11.486%` debt while retaining strong tracking and
communication savings.  The next bounded probe should append the four
three-formation subsets and the all-formation trust-0.30 action.  This tests
coordination order directly without a new trust or duration sweep.
