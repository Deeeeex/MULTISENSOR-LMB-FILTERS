# V81 predictive soft-return recovery design

V80 separates the scales: small global balancing improves X36 recovery, while
every positive global correction worsens M24.  The shared method must therefore
select between qualitatively different recovery actions from the current
state, rather than encode scale-specific constants.

V81 keeps the same V75-safe first-round pulse and introduces one frozen action
bank for both scales:

1. ordinary reference recovery `R`;
2. global reliability-balanced corrections at
   `alpha = 0.02, 0.05, 0.10`;
3. intervention-local soft returns at
   `beta = 0.10, 0.25, 0.50, 0.75`.

The local action changes only receiver rows whose residual sender was replaced
by the first-round pulse.  It restores the incumbent residual edge with weight
`beta * 0.05` and returns the unused residual weight to receiver self.  The
dominant `0.70` sender, every topology edge, and the attempted message count
remain unchanged.  This directly controls the affine forcing at the source of
the intervention instead of perturbing every node as V79 and V80 do.

Recovery is a two-step causal model-predictive procedure.  At each round, all
eight actions are applied virtually to the current candidate posterior with
the formal mixture-aware KLA receiver.  Each output is compared with the next
state of a parallel reference arm using V77 centered energy; the minimum is
selected, with smaller intervention magnitude as the tie break.  The selected
posterior becomes the input to the next recovery round, where the bank is
evaluated again.  This uses no target truth, future measurement, or future
link page and requires no learned model.

The source mechanism gate is deliberately strong: centered energy must be
non-increasing from the safe pulse through both recovery rounds on historical
and receiver-aligned routes for both M24 and X36.  Passing this gate would
authorize the first closed-loop tracking experiment for the V75--V81 method;
it would not itself prove tracking gain.
