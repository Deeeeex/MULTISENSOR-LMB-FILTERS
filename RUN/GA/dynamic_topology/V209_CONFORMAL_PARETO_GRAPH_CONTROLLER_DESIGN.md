# V209 conformal Pareto-safe formation-graph controller

## Method decision

V208 asks whether a truth-free formation-graph representation carries enough
signal to rank posterior communication actions.  If that representation gate
passes, the deployable controller will not treat initialization-ensemble
spread as a safety certificate.  V209 will use a trajectory-level one-sided
conformal calibration layer around a multi-output graph value model.

The controller keeps the structured action family established by V206--V208:

1. no intervention;
2. retain a formation-level posterior withholding decision;
3. release one formation to its ordinary full-posterior route; and
4. apply one complete-label, mixture-aware residual KLA update when the
   receiving formation already supports that label.

The graph network estimates the consequence of an action.  It cannot override
physical reachability, the communication ledger, cooldown, the one-action
page cap, or the supported-label rule.

## V208 representation gate

The completed X36 seed-211 development bank contains `831` executable actions
over eight pages.  A complete-page leave-one-out ridge readout over frozen
two-round graph representations reaches strict-action AUC `0.7839`, top-1
strict recovery on `3 / 8` pages, and selects two actions that are both
strictly positive.  An action-only readout reaches AUC `0.7758` and top-1
recovery on `2 / 8` pages.  The graph ranking advantage is therefore modest;
the important requirement is calibrated abstention, not an unsupported claim
that graph context alone solves selection.

Aggregate and receiver-formation E-OSPA targets have Spearman correlation near
`0.71`, while network-worst E-OSPA and RMSE reach only `0.23` and `0.12`.
V209 therefore treats tail prediction as the binding representation problem.
The result passes a proposal-model gate only.  It does not authorize an online
controller, because page 76 has no jointly positive immediate action even
though its teacher action belongs to a strong recursive sequence.

Existing paired recursive ablations make the target distinction causal rather
than hypothetical.  On the unmodified V99 page-77 state, the F1 `[7,7]`
label action has exactly zero immediate E-OSPA, RMSE and consensus effect.  On
the policy-induced state with the same V204 prefix, deleting only that action
leaves pages 72--76 numerically identical, while retaining it yields no effect
at page 77 and then improves E-OSPA by `1.367%` and `1.373%` at pages 78 and
79.  Its three-page return improves mean E-OSPA by `0.909%`, RMSE by `1.397%`
and consensus by `2.951%`, with `2.708%` attempted-byte saving still remaining
relative to static full-payload routing.  No immediate classifier can recover
this action from its target sign.

The formation-release action is also intrinsically vector-valued.  Comparing
the existing V206 release branch with the same V204 label-action continuation
over pages 72--74, releasing F5 improves RMSE by `0.097%`, consensus by
`0.234%`, and F5 RMSE by `1.532%`, but worsens mean E-OSPA by `0.548%` and F5
E-OSPA by `3.729%`.  It adds `19,520 B` in that window while retaining
`4.325%` saving relative to static.  Release must therefore compete with
no-op under the full risk vector; support-gap detection cannot force it.

## Vector value and explicit risk budget

For observable state `x`, feasible action `a`, and protected metric `j`, let

`Delta_j(x,a)`

denote improvement relative to the same-state no-op continuation.  The value
vector contains mean E-OSPA, mean RMSE, consensus, receiver-formation E-OSPA
and RMSE, minimum affected-sensor E-OSPA and RMSE, and network-worst E-OSPA
and RMSE.  Attempted-byte saving is computed exactly by the communication
ledger rather than learned.

Communication feasibility is cumulative, not a demand that every repair send
fewer bytes than no-op.  Dynamic routing first earns attempted-byte credit
relative to static full-payload routing; a repair may spend part of that
credit, as the F1 action does, provided the protected reserve and cumulative
net saving remain nonnegative.  This preserves the actual research objective
of lower total communication while allowing informative payloads to replace
redundant ones.

The regression heads operate on the bounded monotone target
`z_j = tanh(log(error_reference/error_candidate))`.  Conformal residuals and
lower bounds are formed in this transformed space; the lower bound is then
mapped back to percent improvement by
`100 * (1 - exp(-atanh(z_lower)))`.  Monotonicity preserves coverage, the zero
boundary and every admission comparison while preventing one extreme sensor
tail from dominating the fit.

Zero regression on every tail is retained as a strong reporting label, but it
is not the only admissible definition of a useful action.  Before calibration
data are opened, each protected target receives a tolerance `tau_j`:

- aggregate E-OSPA and RMSE require positive improvement;
- consensus may not regress;
- receiver-formation, cross-formation and network-worst targets may use a
  small, explicitly reported negative tolerance;
- the minimum affected-sensor value remains a reported diagnostic and receives
  a broad, scale-free catastrophe cap rather than a zero-regression veto; and
- exact attempted-byte saving must remain nonnegative.

This turns the previously over-strict all-zero tail gate into an auditable
risk budget.  The tolerances are shared across M24/X36 and scene styles and
cannot be tuned on recursive evaluation outcomes.

This distinction is required by the F1 branch rather than chosen for easier
passing.  Its three-page receiver-sensor E-OSPA gains are all about `4.83%`,
but two of six receiver RMSE gains are `-6.059%` and `-3.491%`.  At the same
time, F1 formation RMSE improves `1.287%`, network-worst RMSE does not regress,
and the aggregate E-OSPA/RMSE/consensus vector is strongly positive.  Requiring
every node to improve would reject useful error redistribution.  Conversely,
the dense bank contains affected-sensor RMSE losses exceeding `1000%`; the
catastrophe cap is still needed to reject genuine local collapse.  The strong
zero-tail label remains in all tables so this relaxation is visible.

## Dense-immediate and sparse-recursive value model

Let `d_j(x,a)` be the current-page transformed gain and `q_j^H(x,a)` the
same-state no-op-relative gain over a fixed `H=3` continuation.  The shared
graph encoder has a dense auxiliary head for `d_j` and a sparse propagation
head for the residual

`rho_j^H(x,a) = q_j^H(x,a) - d_j(x,a)`.

The deployable value is `q_hat_j^H = d_hat_j + rho_hat_j^H`; selection never
uses the immediate head by itself.  This decomposition lets all shortlisted
actions train local posterior sensitivity while reserving expensive recursive
rollouts for delayed propagation.  Losses are balanced by complete trajectory
and action mode, so hundreds of immediate label rows cannot numerically erase
the smaller recursive release and no-op comparisons.

The sparse query set is the deterministic union of top-ranked actions,
high-uncertainty actions, one representative per semantic mode, and actions
actually visited by the current selector.  The same receiver-formation and
label cannot be queried on consecutive pages; source changes do not reset this
one-page semantic cooldown.  This preserves the useful t=76/t=78 F3 refresh
while removing the empirically redundant t=78/t=79 repeat.

## Simultaneous trajectory-level conformal lower bound

Training, calibration and evaluation are split by complete scene-seed
trajectories.  For a calibration trajectory `k`, define the nonconformity
score

`S_k = max_(t,a,j) (h(hatDelta_j(x_kt,a)) - h(Delta_j(x_kt,a))) / s_j(x_kt,a)`,

where the maximum covers every evaluated page, every feasible candidate and
every protected learned target.  `s_j` is a positive scale with a frozen
floor; it may use ensemble dispersion, but dispersion alone is not a
certificate.  Let `q_alpha` be the finite-sample conformal quantile of the
trajectory scores.  The online lower bound is

`h(L_j(x,a)) = h(hatDelta_j(x,a)) - q_alpha s_j(x,a)`,

where `h` is the bounded log-ratio transform above and `L_j` is obtained by
its monotone inverse.

Under exchangeability of complete calibration and evaluation value
trajectories generated by the same frozen state-collection protocol, the
usual split-conformal rank argument gives

`P[Delta_j(x_t,a) >= L_j(x_t,a) for every t, a and j] >= 1 - alpha`.

The maximum is taken before calibration, so the event is simultaneous over
the action bank.  Consequently, choosing an action after reading all predicted
lower bounds does not introduce the within-page post-selection gap that an
independently calibrated per-row interval would have.  The guarantee is
deliberately at the trajectory level; treating correlated pages or candidates
as independent calibration examples is forbidden.

## Pareto admission and selection

At each page the deterministic projector first removes physically impossible,
unsupported, over-budget or cooldown-violating actions.  A remaining action
is admitted only if every learned lower bound exceeds its frozen tolerance and
its exact byte saving is nonnegative.  Among admitted actions, the controller
maximizes the minimum normalized margin

`min_j (L_j - tau_j) / scale_j`,

with exact byte saving used only as a secondary tie-break.  If no action is
admitted, the controller chooses no-op.  This rule prevents a large mean gain
from purchasing an unreported sensor-tail loss and avoids a hand-tuned scalar
mixture of tracking, consensus and communication objectives.

## Data and promotion protocol

1. V208 complete-page holdout on one opened X36 trajectory is only a
   representation gate.  It cannot train or calibrate V209.
2. Dense immediate action values are generated on disjoint radial, convoy and
   relay M24/X36 development trajectories.  They initialize the representation
   and proposal ranker only.  The finite-horizon head learns a sparse residual
   over that immediate auxiliary target, so a zero-immediate but delayed-positive
   action remains representable.  Recursive targets are queried for
   high-ranked, high-uncertainty, mode-diverse and policy-visited actions,
   always paired with the same-state no-op continuation.
3. Recursive target generation is iterative: roll out the current frozen
   selector, collect the states induced by its own earlier actions, query a
   bounded action subset at those states, and refit.  A semantic cooldown
   removes repeated receiver-formation/label repairs before ranking.  This is
   required because an immediately attractive repeat may spend credit while a
   different action has the delayed value.
4. Model fitting and conformal calibration use disjoint complete trajectories.
   The calibration manifest, `alpha`, target scales and risk tolerances are
   frozen before recursive evaluation.
5. A first promotion requires paired static-versus-controller gains in mean
   E-OSPA, RMSE, consensus and attempted bytes on unseen M24 and X36 seeds.
6. The frozen controller then transfers without retuning to convoy and relay;
   crossing, braided handover, target overlap, merge-split and curved corridor
   remain stress tests, followed by X48 scale extrapolation.

The first sparse recursive check is deliberately smaller than a new exhaustive
bank.  It reuses the completed V206 action trajectory and runs two paired
delete-one-action continuations:

- delete the t=73 F6 precision-refresh action while retaining the identical
  t=72 prefix, then score t=73--75; and
- delete the t=76 F3 repair while retaining the identical t=72--75 prefix and
  the same forced t=77/t=78 continuation, then score t=76--78.

Each pair must reproduce its common prefix before a return is accepted.  The
three-page vector return is measured relative to the delete-action branch and
reports aggregate tracking, consensus, receiver and sensor tails, the exact
incremental bytes, and remaining saving relative to static full-payload
routing.  These two probes test the delayed-value diagnosis; they are not
training or calibration examples for a final model.

## Claim boundary

The conformal statement covers the registered counterfactual value targets
under trajectory exchangeability; it is not a proof that the simulator equals
a real sensor network.  In particular, calibration on states collected by the
base route does not automatically cover the recursively shifted state
distribution induced by a new controller.  A closed-loop guarantee would
require calibration trajectories collected under the same frozen controller
or a separate sequential risk-control argument.  V209 therefore uses the
bound as a protected action-selection layer and still requires independent
paired recursive evaluation.  The residual label operator remains the
repository's componentwise powered-GM approximation and is not claimed to be
exact KLA for arbitrary Gaussian mixtures.

## Primary literature anchors

- Kiyani et al., *Decision Theoretic Foundations for Conformal Prediction:
  Optimal Uncertainty Quantification for Risk-Averse Agents*, ICML 2025,
  <https://proceedings.mlr.press/v267/kiyani25a.html>, supplies the general
  connection between conformal uncertainty sets and max-min risk-averse
  decisions.  It does not supply the LMB action space or the trajectory-wide
  score used here.
- Dietterich and Hostetler, *Conformal Prediction Intervals for Markov Decision
  Process Trajectories*, arXiv:2206.04860,
  <https://arxiv.org/abs/2206.04860>, gives whole-trajectory coverage for a
  fixed control policy.  Its fixed-policy condition is why V209 explicitly
  refuses to transfer a base-route calibration guarantee to a new recursive
  controller without additional evidence.
