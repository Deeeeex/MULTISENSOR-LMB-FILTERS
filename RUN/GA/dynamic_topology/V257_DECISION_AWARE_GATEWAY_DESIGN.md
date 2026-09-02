# V257 decision-aware gateway diagnostic

## Question

V256 established a clean negative result: its strongest ridge model is worse
than a seed-blind mean predictor by 3.636% across seven tracking outcomes and by
9.076% on receiver-formation RMSE.  That result does not reveal whether the
failure comes from regressing noisy absolute outcomes or from an observation
that omits the posterior compatibility needed to value a gateway.

V257 separates these explanations before opening any new data.  It uses the
same 42 windows, 492 executable actions, seven training seeds, physical action
banks and deterministic communication masks as V256.

## Two-by-two diagnostic

The loss axis compares the V256 multi-output ridge with a joint-safe ridge.  An
action is jointly safe only when realized network E-OSPA, RMSE and consistency
all improve, both formation tails stay within the registered 2% tolerance, the
receiving formation's E-OSPA does not regress, and its RMSE improves by more
than 0.5%.  The safety model estimates this binary event and ranks actions by
its score; it does not regress the eight raw outcomes.

The information axis compares the costed 32-dimensional compact representation
with a 44-dimensional upper bound that additionally exposes six exact current-
posterior pairwise quantities for both candidate and incumbent edges: existence
gain/gap, precision gain/gap, normalized state discrepancy and active-label
overlap.  These quantities contain no truth or future outcome, but their current
communication cost is not registered.  Therefore the rich mode is a mechanism
diagnostic only, not a deployable policy.

## Evaluation

Every configuration is evaluated by leaving out one complete seed.  In each
held window, the action with the largest score is compared with the uniform
safe-action prevalence of that same window.  This produces a top-action safe
lift that cannot be inflated by windows containing more candidates.  The
diagnostic also reports how often a thresholded policy acts, the precision of
those actions, and selected-or-reference mean outcomes.

A feature/loss combination shows actionable ranking information only if its
safe lift is at least five percentage points, lift is positive in at least five
of seven held seeds, and a thresholded policy selects at least seven actions at
no less than 50% realized precision.  Passing this diagnostic does not open
calibration or holdout data.  Compact-only success motivates a better ranking
model; rich-only success motivates a fixed-byte posterior compatibility sketch.
Failure of both stops local gateway learning and redirects the method design to
a different action scale or horizon.  GNN, X36 and paper claims remain sealed.

## Frozen result

The within-window uniform safe prevalence is 10.9%.  The strongest compact
multi-output ranker reaches 14.3%, a lift of only 3.3 percentage points, and is
positive in four of seven held seeds.  Its thresholded policy selects 16 of 42
windows but only two selections are realized-safe; mean consistency regresses.
The compact joint-safe model reaches only a 1.0-point lift and acts once.

Exact pairwise posterior information does not repair the failure.  Its best
top-action lift is 1.0 point.  The most conservative rich multi-output model
selects 15 windows at 20% precision, with negative mean E-OSPA gain; the rich
joint-safe models abstain completely.  Neither information mode passes the
predeclared diagnostic.

The local `one directed arc for H=3` learning route is therefore stopped before
calibration.  The evidence points to a weak and realization-sensitive action,
not merely a poor compact synopsis or the wrong regression loss.  A successor
must change the intervention itself: a longer event-defined hold, coordinated
cut reinforcement, or another action scale with a stronger deterministic link
to consensus transport.  Increasing model capacity on the same labels is not
authorized.
