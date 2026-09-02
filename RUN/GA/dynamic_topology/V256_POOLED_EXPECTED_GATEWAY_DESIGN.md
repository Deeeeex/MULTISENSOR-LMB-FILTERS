# V256 pooled expected local gateway value

## Decision problem

V252 shows that useful gateway replacements exist, but the same observable
action can have different three-page outcomes under different measurement and
delivery realizations.  V253 incorrectly compressed eight consequences into a
single minimum label; the first V255 model instead took the coordinate-wise
worst prediction from three seed-specific regressions and consequently
abstained in 17/18 windows.  Both try to make a deterministic statement about
one stochastic future from too little repeated evidence.

The deployable decision is an average-effect problem.  At time `t`, the state
`x_t` contains only current local LMB summaries, geometry, link reliability and
causal route history.  For a one-arc replacement `a`, let

`Y_k(a, t) = L_k(V242, t:t+2) - L_k(a, t:t+2)`

be the paired percentage gain for outcome `k`.  Shared measurements, delivery
uniforms and filter RNG make this a low-variance paired contrast inside each
seed.  V256 estimates `E[Y_k | x_t, a]`; it does not claim to predict the exact
realized `Y_k` of the next three pages.

## Action and feature representation

V242 remains the fallback.  A candidate changes exactly one directed physical
gateway while preserving its source/receiver formation roles, the formation
tree, local cycles, KLA weights and `N + 2(F-1)` posterior-message count.  The
V255 teacher supplies the reference plus two ranked candidates for every
directed formation slot.

Each directed formation-tree slot contributes up to two physical one-arc
alternatives.  A window may contain fewer than twelve actions when the current
physical graph offers only one alternative in a slot, or none when that slot
has no executable replacement.  Such a slot is non-actionable for that window;
its absence does not invalidate real alternatives in other slots, and edges are
never fabricated to balance the dataset.  Ridge fitting gives every seed equal
total weight, every registered anchor equal weight within a seed, and divides
one anchor's weight uniformly over its observed actions.

The same assembler is registered for three disjoint roles: the seven-seed
training pool, seeds 1311--1312 for calibration, and seed 1306 for the
development holdout.  Each role has a distinct seed registry and output
artifact, while action reconstruction, observable features and deterministic
communication projection are identical.  The untouched complete-episode seed
1305 is not an assembler role and cannot enter any labeled local dataset.

Each edge has a 16-dimensional compact observable summary.  The action feature
is

`phi(x,a) = [edge_candidate - edge_incumbent, edge_incumbent]`.

The difference term describes the intervention and the incumbent term describes
the state in which that intervention is made.  This 32-dimensional transform is
equivariant to sensor and formation relabeling and contains no numeric identity,
truth or future outcome.

## Expected-outcome model selection

The seven training seeds are `1302--1304` and `1307--1310`.  A single
multi-output ridge is fitted to all training rows with equal seed weight.  The
regularization penalty is selected by leaving out one entire seed at a time.
The fit may use all topologically valid one-arc outcomes, but model selection
is evaluated only on held-seed actions that pass the deterministic
communication projection, because other actions cannot be selected online.
For each fold, squared prediction error is normalized by the supported
training-fold standard deviation of that outcome and then averaged equally
over seven tracking outcomes: network E-OSPA, network RMSE, consistency, two
formation-tail coordinates and two receiver-formation coordinates.  Fold
errors are averaged with equal weight per held seed, so a seed with more
communication-feasible candidates cannot dominate the decision.

The attempted-byte coordinate is still reported but cannot choose the penalty
or admit an action.  Communication admission is a deterministic projection.
At the decision page, the current posterior sizes are evaluated on the current
physical V240 two-input causal route, the V242 minimum backbone and every
one-arc replacement.  Current per-node sizes are held constant only for the
three-page byte estimate, and the 408 B controller exchange is charged once
for the hold.  A local action may consume at most 20% of the estimated
dense-causal-versus-V242 saving, so at least 80% of that saving is retained.
The fixed static route remains the end-to-end experimental baseline; it is not
used as an online guarantee when some of its edges are physically unavailable.
This keeps stochastic realized payload prediction from being mistaken for a
hard budget guarantee.

The learned representation is considered informative only if its
leave-one-seed-out error is lower than a seed-blind training-mean predictor both
on the seven-outcome mean and on the primary receiver-RMSE coordinate.  Failure
of this comparison stops outcome-conditioned learning before calibration; it
does not justify a more complex GNN.

## Independent uncertainty calibration

After the feature transform and ridge penalty are frozen, seeds 1311 and 1312
are opened only for uncertainty calibration.  For every predicted tracking
gain, the relevant one-sided residual is `prediction - realized gain`: positive
values measure overconfidence.  Because the online policy selects one action
from a candidate set, V256 first takes the maximum residual over all
communication-feasible actions in each window.  It then takes the nearest-rank
80th percentile of the six window maxima within each seed and the nearest-rank
80th percentile across seed summaries.  The resulting nonnegative margin is
subtracted from each prediction.

This is a deliberately empirical development margin.  Two calibration seeds do
not provide a meaningful distribution-free 80% coverage guarantee at the seed
level, so the paper may not present it as conformal coverage.  Its purpose is to
separate penalty selection from abstention calibration and to expose whether
the current data volume is adequate.  A formal coverage result would require a
larger predeclared calibration set.

## Online decision

The deterministic topology and communication projection runs first.  Among
feasible candidates, an action is admissible only if the calibrated lower
bounds predict positive network E-OSPA, RMSE and consistency gains, both
formation-tail coordinates stay above the registered `-2%` tolerance, the
receiver formation does not regress, and receiver RMSE exceeds `0.5%`.  The
policy selects the largest receiver-RMSE lower bound and otherwise retains
V242 for all three pages.

Under a coordinate-wise error event `|hat Y_k - E[Y_k|x,a]| <= epsilon_k`,
tightening each constraint by `epsilon_k` guarantees feasibility with respect
to the conditional expected outcomes.  The chosen action's conditional
receiver-RMSE regret is at most `2 epsilon_R` relative to the best action that
retains the same safety margin.  Physical feasibility, strong connectivity,
message count and communication credit do not depend on this learned error
event.

## Evidence order

1. Complete only seeds 1307--1310 under the frozen V255 teacher protocol.
2. Select the ridge penalty from all seven training seeds without reading
   1311, 1312, 1306 or 1305.
3. Open 1311--1312 only to compute the fixed one-sided margins.  Require at
   least one realized safe selection in each calibration seed.
4. Open seed 1306 exactly once.  At least two of six windows must select a
   realized safe action, the aggregate E-OSPA/RMSE/consistency gains over V242
   must all be positive, formation regression must stay within 2%, and the
   charged incremental bytes must stay within 2% of V242 before a complete
   episode is authorized.
5. Use untouched seed 1305 for the complete M24 policy only after step 4.
6. Freeze the model and controller before X36.  X36 and later crossing,
   merge-split and curved-corridor scenes are independent generalization tests,
   not tuning data.

GNN escalation remains unauthorized.  It requires both positive held-out local
selection evidence and a repeatable nonlinear residual that ridge cannot
explain.

## Frozen training outcome

The seven-seed dataset contains 42 windows and 492 physically executable
one-arc actions.  Every retained action passes the deterministic communication
projection; windows contain 9--12 actions because seed 1304 at `t=140` has no
physical replacement for one directed slot.  That missing action remains
absent rather than being fabricated.

Leave-one-seed-out selection chose `lambda=100`.  Even at this strongest
regularization, the 32-dimensional compact model is worse than the seed-blind
training-mean predictor by 3.636% over the seven tracking outcomes and by
9.076% on receiver-formation RMSE.  Calibration is therefore not authorized,
and seeds 1311--1312 remain unopened.

This result rejects absolute multi-output regression from the current compact
edge synopsis; it does not yet distinguish an unsuitable loss from missing
observations.  Any successor must use only these training seeds to compare
decision-aware within-window ranking against a fixed-dimensional posterior
compatibility enrichment.  A nonlinear model is justified only if compact
features retain cross-seed decision signal that a linear ranker cannot capture.
