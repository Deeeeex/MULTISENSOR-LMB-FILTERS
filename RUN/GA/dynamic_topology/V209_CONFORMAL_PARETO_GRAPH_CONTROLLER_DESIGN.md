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

The first newly generated delete-action pair confirms the same phenomenon for
F6.  The V206 and no-t=73 branches reproduce the complete t=72 prefix exactly.
At t=73 the action has zero E-OSPA gain, but its E-OSPA gain becomes `0.940%`
and `1.553%` on the next two pages.  Over t=73--75 it improves mean E-OSPA by
`0.818%`, RMSE by `12.748%`, consensus by `2.831%`, and F6 formation RMSE by
`55.479%`; minimum receiver-sensor, network-worst and cross-formation tails do
not regress.  It adds `90,520 B` relative to no-op while the controller still
retains `4.544%` attempted-byte saving relative to static full-payload routing.
Thus H=3 recovers both the zero-immediate F1 set repair and the deferred F6
precision refresh under one target definition.

The dense auxiliary page regenerated on the exact policy-induced t=73 state
also recovers the F6 teacher key.  Its immediate E-OSPA, RMSE and consensus
gains are `0.000%`, `13.166%` and `0.537%`, compared with `0.000%`, `12.047%`
and `0.387%` on the base-route state.  The policy state shifts magnitude but
not the immediate target semantics: E-OSPA still appears neutral even though
the same action has positive three-page E-OSPA value.  This is the intended
division of labor between the dense immediate auxiliary head and the sparse
recursive residual head.

The second newly generated delete-action pair is an equally important
rejection example.  Retaining the t=76 F3 `[19,13]` repair improves the
three-page mean E-OSPA and RMSE by `0.351%` and `2.348%`, but worsens mean
consensus by `0.683%` and terminal consensus by `4.374%`; its minimum
receiver-sensor E-OSPA also regresses by `0.352%`.  The action spends
`97,384 B` relative to no-op and leaves only `1.419%` saving relative to
static full-payload routing.  The common prefix matches exactly, so this is
not trajectory drift.  V209 must reject the action despite its positive
tracking coordinates: a scalar tracking score or immediate E-OSPA/RMSE
classifier would make the wrong decision, whereas the vector lower-bound
admission rule falls back to no-op because the consensus coordinate is
negative.

Policy-state transfer closes the naive small-top-K route.  When the base page
at the same time is excluded, an immediate-target readout ranks the t=73 and
t=77 delayed-positive teacher sources only `32 / 103` and `48 / 104` with the
graph representation (`13` and `9` action-only); none enters global top-5.
Adding 25 truth-free H=3 propagation descriptors but still training on the
immediate target does not fix this target mismatch: the graph ranks become
`33` and `50`.  Propagation features therefore belong in the recursive
residual head, not in an immediate-value proposal score.

The first structural alternative is more promising.  A development-only
V211 rule selected the formation with maximum observable `need_max`, then
kept the top three source-label actions for each of precision refresh,
receiver existence deficit and observation handover.  The rule covered the
delayed-positive t=73 F6 and t=77 F1 actions and excluded the vector-negative
t=76 F3 action, but retaining exactly one formation would become increasingly
narrow as the number of formations grows.

V213 freezes the scale-aware replacement before opening new trajectories.
It retains every formation with `need_max >= 0.90 max(need_max)`, capped at
two formations, then takes the union of the top three actions in the same
three modes.  It also adds a release action only when that formation is
currently eligible to return to the ordinary full-posterior route.  The
resulting registered set contains at most `18` label actions and two releases.
On the already opened states it selects F6+F1 at t=73 and F1 at t=76/t=77,
giving label-set sizes `16 / 8 / 8`, positive coverage `2 / 2`, and negative
exclusion `1 / 1`.  These figures are a development contract check, not a
held-out result; the thresholds and caps can no longer change in response to
seeds 1301 and above.

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
vector contains mean E-OSPA and RMSE, window and terminal consistency,
receiver-formation E-OSPA and RMSE, minimum affected-sensor E-OSPA and RMSE,
network-worst E-OSPA and RMSE, and minimum-formation E-OSPA and RMSE.
Attempted-byte saving is computed exactly by the communication ledger rather
than learned.

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
data are opened, V213 freezes the 12-target tolerance vector, in target order,
as

`[0, 0, 0, 0, -1.5, -1.5, -5, -10, -1.5, -1.5, -1.5, -1.5]%`.

Thus aggregate E-OSPA/RMSE and both consistency coordinates may not regress;
receiver-formation, network-worst and minimum-formation tails have a visible
`1.5%` redistribution budget; minimum affected-sensor E-OSPA has a `5%` cap;
and affected-sensor RMSE has a `10%` catastrophe cap.  In addition, at least
`1%` exact attempted-byte saving relative to static full-payload routing must
remain after the action.  The vector is shared across scales and scene styles
and cannot be tuned on recursive evaluation outcomes.

This turns the previously over-strict all-zero tail gate into an auditable
risk budget while keeping every redistribution visible in result tables.

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
uses the immediate head by itself.  The dense head consumes the cheap current
posterior/action features.  The residual head additionally consumes continuous
H=3 open-loop distance, velocity, covariance and Mahalanobis descriptors from
the known motion model.  These descriptors read no truth or future
measurement and replace V189's invalid binary distance veto with learned
evidence.  Their distributed layout costs one `64 B` source synopsis and six
float32 summaries (`24 B`) per receiver: `208 B` for the six-node X36
formations.  This cost is charged exactly and the descriptors are computed
only after structural shortlisting.

The decomposition lets all shortlisted actions train local posterior
sensitivity while reserving expensive recursive rollouts for delayed
propagation.  Losses are balanced by complete trajectory and action mode, so
hundreds of immediate label rows cannot numerically erase the smaller
recursive release and no-op comparisons.

Exhaustively labeling every feasible action with an H=3 rollout is neither
necessary nor computationally credible.  Before recursive fitting, a
truth-free hierarchical proposal rule is frozen.  It first ranks formations
from the observable graph, then forms a mode-diverse source-label union inside
the retained formations.  This maps the physically feasible bank to a small,
possibly variable-size registered set `P_K(x)` with a fixed maximum `K`.
The residual model may rank or abstain inside this set, but it cannot add an
action that the registered structural rule omitted.  `K`, formation score,
mode caps and all tie-breaking are fixed before recursive calibration;
actions outside `P_K(x)` are not assigned a conformal guarantee and cannot be
selected.

The sparse recursive training queries are the deterministic union of
proposal-set actions on training states, high-uncertainty proposal
alternatives, and actions actually visited during iterative state collection.
Training may use this broader set, but calibration evaluates every member of
the frozen `P_K(x)` set on each registered page.

The same receiver-formation and label cannot be proposed on consecutive
pages; source changes do not reset this one-page semantic cooldown.  A
formation release is keyed by its receiving formation and is proposed only
when the current route state marks that release executable.  Cooldown is
applied before mode ranking, allowing the next eligible source-label action
to fill the structural proposal instead of wasting a slot.  The
completed t=76 counterfactual removes the previous assumption that every F3
refresh in the teacher sequence is useful: cooldown controls duplicate
queries, while the vector value gate must still reject an individually
harmful, non-duplicate repair.

## Simultaneous trajectory-level conformal lower bound

Training, calibration and evaluation are split by complete scene-seed
trajectories.  For a calibration trajectory `k`, define the nonconformity
score

`S_k = max_(t,a in P_K(x_kt),j) (h(hatDelta_j(x_kt,a)) - h(Delta_j(x_kt,a))) / s_j(x_kt,a)`,

where the maximum covers every evaluated page, every member of the frozen
proposal set and every protected learned target.  `s_j` is a positive scale
with a frozen floor; it may use ensemble dispersion, but dispersion alone is
not a certificate.  Let `q_alpha` be the finite-sample conformal quantile of
the trajectory scores.  The online lower bound is

`h(L_j(x,a)) = h(hatDelta_j(x,a)) - q_alpha s_j(x,a)`,

where `h` is the bounded log-ratio transform above and `L_j` is obtained by
its monotone inverse.

Under exchangeability of complete calibration and evaluation value
trajectories generated by the same frozen state-collection protocol, the
usual split-conformal rank argument gives

`P[Delta_j(x_t,a) >= L_j(x_t,a) for every t, a in P_K(x_t) and j] >= 1 - alpha`.

The maximum is taken before calibration, so the event is simultaneous over
the registered proposal set.  Consequently, choosing an action after reading
all of its predicted lower bounds does not introduce the within-page
post-selection gap that an independently calibrated per-row interval would
have.  This reduces recursive labeling from the full feasible bank to a small
fixed `K` without pretending to cover actions that were never calibrated.
The guarantee is deliberately at the trajectory level; treating correlated
pages or candidates as independent calibration examples is forbidden.

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
   action remains representable.  The proposal-set size and diversity rule are
   registered before recursive calibration.  Training targets are queried for
   high-ranked, high-uncertainty, mode-diverse and policy-visited actions,
   always paired with the same-state no-op continuation.
3. Recursive target generation is iterative: roll out the current frozen
   selector, collect the states induced by its own earlier actions, query a
   bounded action subset at those states, and refit.  A semantic cooldown
   removes repeated receiver-formation/label repairs before ranking.  This is
   required because an immediately attractive repeat may spend credit while a
   different action has the delayed value.
4. Model fitting and conformal calibration use disjoint complete trajectories.
   On calibration trajectories, every member of the frozen `P_K(x)` set is
   evaluated so the trajectory maximum covers every action the online
   controller may later choose.  The proposal rule, calibration manifest,
   `alpha`, target scales and risk tolerances are frozen before outcomes are
   opened.
5. A first promotion requires paired static-versus-controller gains in mean
   E-OSPA, RMSE, consensus and attempted bytes on unseen M24 and X36 seeds.
   The frozen per-scale thresholds are respectively `5%`, `5%`, `2%` and
   `1%`; a below-threshold controller may remain in the current-best table
   with its evidence boundary but cannot become a paper conclusion.
6. The frozen controller then transfers without retuning to convoy and relay;
   crossing, braided handover, target overlap, merge-split and curved corridor
   remain stress tests, followed by X48 scale extrapolation.

The registered V213 manifest groups complete trajectories across scale and
style.  Training seeds are `1301/1303`, architecture selection uses
leave-one-complete-training-trajectory-out, and calibration seeds are
`1409/1423` with trajectory-level miscoverage `alpha=0.10`.  Evaluation uses
`1511/1523`; stress uses `1601/1607`; and X48 extrapolation uses `1709/1721`.
The main radial, convoy and relay set is evaluated at both M24 and X36.  The
five stress styles are also paired across M24/X36, giving ten stress presets,
and X48 radial is the seventeenth registered preset.  Each trajectory exposes
two truth-free local maxima of formation need separated by at least 12 pages.
No page, scene-seed trajectory, or seed-211 mechanism anchor crosses a split.

## V212--V213 freeze checkpoint

V212 starts at the captured policy-induced t=73 state rather than replaying
the full eight-page prefix.  Its no-op and known F6 `[7,5]` branches both
reproduce the completed H=3 source windows with zero E-OSPA, RMSE and
consistency difference and exact attempted bytes.  The short continuation
recovers the known `+0.818%` E-OSPA, `+12.748%` RMSE and `+2.831%` consistency
return.  This validates the faster recursive-label path only; its same-state
static saving is `5.738%` and must not be confused with the older full-window
`4.544%` figure.

The V213 preflight then applies the frozen proposal and risk contracts without
running a new tracking outcome.  Every X36 label proposal is charged
`64 + 24*6 = 208 B`; total descriptor charges are `3328 / 1664 / 1664 B` at
t=73/t=76/t=77.  The frozen risk budget admits t=73 and t=77, rejects the
vector-negative t=72 release and t=76 repair, and preserves at least `1%`
static communication credit in both admitted cases.  These four seed-211
anchors are prohibited from training or calibration.  The next admissible
evidence is therefore recursive data collected on the registered fresh
M24/X36 training trajectories under the generic V99 base route.

The completed sparse recursive check is deliberately smaller than a new
exhaustive bank.  It reuses the V204/V206 teacher trajectories and two paired
delete-one-action continuations.  Four same-prefix branches now provide the
minimal protocol test:

- t=72 F5 formation release is vector-negative and maps to no-op;
- t=73 F6 precision refresh is strictly positive over t=73--75;
- t=76 F3 repair has positive tracking but negative consensus and maps to
  no-op; and
- t=77 F1 set repair is aggregate-positive with a bounded receiver-sensor
  tail and tests the explicit risk-tolerance path.

Each pair must reproduce its common prefix before a return is accepted.  The
three-page vector return is measured relative to the delete-action branch and
reports aggregate tracking, consensus, receiver and sensor tails, the exact
incremental bytes, and remaining saving relative to static full-payload
routing.  All four prefixes match.  These four rows establish target semantics
and unit-test the data path, but are far too few and too development-specific
to train or calibrate a final model.

## V214 direct-graph pivot

V213 assumed that the generic online V99 selector could serve as the no-op
trajectory on every training page.  The completed X36 seed-211 H=8 screen
contradicts the practical side of that assumption: the paired static arm took
`251.51 s`, whereas the V99 online arm took `1394.69 s`, or `5.545x` as long.
V99 repeatedly evaluates posterior counterfactuals for each formation, so a
complete 133-page scan followed by an identical capture replay would turn the
runtime controller into the dominant computational cost.  This does not
invalidate V99 as a mechanism teacher, but it makes V99 an unsuitable
deployable base and an inefficient state-collection policy.

V214 therefore keeps the V213 complete-trajectory split, H=3 vector targets,
proposal caps and trajectory-level risk calibration, but changes the causal
base.  Each page now executes only the current-physical fixed spliced-cycle
reference route and constructs one V208 observable formation graph.  The
posterior-derived graph is diagnostic at collection time and cannot change the
route.  All focus-page predecision posteriors are captured in one pass; two
local maxima of `need_max`, separated by at least 12 pages, are selected after
the truth-free trajectory is complete, and only those two states are persisted.
There is no second full-trajectory replay and no online posterior
counterfactual enumeration.

The deployable action space becomes a direct graph decision rather than a
wrapper around V99: retain full payload, withhold one receiver formation's
complete cross-formation posterior for one page, execute one supported
complete-label KLA repair, or combine withholding with one repair while
preserving the reference route.  This correction deliberately does not revive
moment-matched light posteriors: every transmitted label keeps its complete
Gaussian mixture.  A formation cannot be withheld on consecutive pages, and
the following pages return to the full-payload route, giving the effective
label graph an explicit three-page recovery window.  Proposal features,
control messages and complete-label responses are all charged in the
attempted-byte account.  A later safe graph-codebook option may change the
route only after the same rolling-connectivity projection; payload and repair
learning do not receive permission to bypass that projection.  V99 remains an
offline action-value teacher and mechanism upper bound, not a runtime input.
Consequently, fresh V214 results must be compared directly with the same-state
full-payload reference, and the current-best table is unchanged until a paired
controller result exists.

The first registered training trajectory now closes the collection path.
`m24-formation-fov / seed 1301` scanned pages `55:133` in `1689.83 s`, selected
truth-free need maxima at `t=119` and `t=131` with scores `2.18074` and
`4.48701`, and persisted two readable 24-sensor caches with exactly two past
topology pages.  The selected windows meet the frozen 12-page separation and
the runtime performed zero posterior counterfactual enumerations.  This is
state-collection evidence only.

Both registered H=3 withholding screens are now complete.  At `t=119`, F1
withholding improves E-OSPA/RMSE/window consensus by `1.074% / 0.041% /
1.093%`, but the next two full-payload pages rebound enough to increase total
attempted bytes by `0.240%`; F2 saves `0.699%` but causes a `19.554%` RMSE
loss; and F3/F4 remove `0.941% / 0.867%` communication with output metrics
exactly unchanged.  At `t=131`, F1 is the first fully eligible direct action:
E-OSPA, RMSE, window consensus and terminal consensus improve by `0.690%`,
`0.645%`, `2.205%` and `2.930%`, attempted bytes fall by `1.254%`, and no
reported sensor or formation tail degrades.  F4 again gives an exactly
output-equivalent `0.960%` saving.  These are training-split action values,
not an online-controller or validation claim.

This evidence rejects a single `need_max` ranking.  Receiver need is a repair
signal, while withholding value depends on a different causal question: is
the incoming posterior redundant or mutually inconsistent with the receiver,
and will the saved current payload reappear as extra labels or mixture
components on the recovery pages?  The observable F1 state at `t=131` is an
extreme case (`need_max=4.487`, mean quality `-1.923`, mean position variance
`3.073`); its only cross-formation reference source has larger quality
advantage and state discrepancy and lower label overlap than at `t=119`.
V215 therefore keeps separate withholding, repair and payload-rebound heads.
The new withholding feature schema combines the receiver node features, the
selected incoming formation-edge semantics, scale-normalized current full-GM
payload complexity and incoming reference-payload share.  No action rule is
changed yet: the two completed windows remain labels for the next frozen
selector rather than thresholds fitted to two examples.

The second M24 training trajectory changes the budget interpretation without
changing that claim boundary.  `seed 1303` selected `t=104` and `t=132`.  At
`t=104`, F2 improves E-OSPA, RMSE, window consistency and terminal consistency
by `1.545% / 1.009% / 3.601% / 0.931%`, with no reported tail degradation,
but spends `0.442%` more bytes than the same-state reference.  Conversely,
several other formation actions across the four registered windows save about
`0.8--1.1%` with exactly unchanged output.  Requiring every individual action
to be byte-nonnegative therefore discards a plausible trajectory-level
composition: harmless actions can earn credit, and a later high-value repair
can spend only that already certified credit.  The exact cumulative ledger,
not an unconditional per-action byte gate, is the correct invariant.

## V216 causal credit-repair composition

V216 implements that composition as a two-stage causal action rather than
preselecting a semantic label from the precommunication graph.  Before the
ordinary page exchange, the graph head may withhold one receiver formation's
cross-formation full posterior for one page.  After the remaining messages
have been fused, a deterministic repair head observes only the current fused
and local posteriors, the current physical/delivery graph and the exact byte
ledger.  It may choose at most one label for which every receiver already has
positive support and a directly reachable source has lower advertised Bayes
risk.  The source response preserves the complete Bernoulli Gaussian mixture,
and each receiver applies the repository's componentwise powered-GM label KLA
with source weight `0.5`.  Zero-support labels remain ineligible because a
geometric average cannot restore them.

This separates three decisions that were previously conflated.  The graph
head decides which full posterior is redundant or conflicting enough to pause;
the postfusion repair head decides which supported label is worth refreshing;
and the byte projector decides whether the synopsis, request and complete
label response fit after locking the frozen `20%` reserve.  Delivery is sampled
from the current link state, fixed formation/source/label teacher identifiers
are absent, and no posterior counterfactual is evaluated online.  The first
paired screen keeps the existing four M24 withholding arms but turns each into
`withhold one formation + causal supported-label KLA`, so it isolates whether
the saved payload can finance a useful repair without expanding the action
bank.  A repair that cannot be paid or delivered reduces automatically to the
underlying one-page withholding action.  H=3 screens remain training action
values; only a later frozen, recursively executed controller can refresh the
current-best table.

The first `seed 1301 / t=131` execution separates affordability from value.
With a network-wide 24-byte synopsis, all four arms produced safe proposals
but none could pay a complete-label response: the synopsis alone cost about
`17.4--17.6 kB`.  Scoping metadata to the withheld formation and its current
common sources reduced each bank to one proposal, and removing the two fields
not consumed by the repair rule (association support and observation
opportunity) produced a 20-byte risk-only record.  The resulting F1 action
paid a `16,576 B` complete-label response, retained `6,120 B` certified credit
on the action page and reduced the full H=3 attempted bytes by `1.374%`.

That affordable repair is a negative value label, not a promoted method.
Relative to the same-state full-payload reference, E-OSPA and RMSE degrade by
`0.187%` and `1.719%`; the minimum formation E-OSPA/RMSE gains are
`-0.628% / -4.350%`.  Consensus nevertheless improves by `2.087%` over the
window and `4.801%` at the terminal page.  The selected source has a positive
advertised Bayes-risk reduction, so this result directly disproves using
source risk as an acceptance certificate.  The analytic rule remains useful
only as a truth-free executable proposal generator.  A separate finite-horizon
repair-value head must distinguish tracking-improving refreshes from
consensus-only or harmful KLA actions, while the deterministic ledger and
support checks continue to own feasibility.  This failed training arm stays
in the repository evidence and does not update the main current-best table.

## V217 operator-consistent overlap diagnosis

The next diagnosis reads the normalizer already used by the live residual
label operator instead of treating the V188 isotropic position score as KLA
itself.  For each receiver/source pair the instrumented fusion kernel now
reports the powered-GM spatial `log eta`, the weighted input existence
log-odds, the fused log-odds and their numerical identity residual.  The
public diagnostic retains the important qualification that this is the
repository's componentwise powered-GM approximation, not exact arbitrary-GM
KLA.

The three X36 actions with independently paired H=3 attribution show why a
single full-state overlap threshold would be wrong:

| action | paired H=3 interpretation | minimum isotropic position compatibility | receivers inside 99% position-support gate | median live `log eta` | median receiver log-odds change |
|:--|:--|--:|--:|--:|--:|
| t=73, F6 `[7,5]` | vector-positive | 0.875109 | 6/6 | -7.33536 | -3.69567 |
| t=76, F3 `[19,13]` | vector-negative | 0.0000746 | 0/6 | -8.81939 | -3.19595 |
| t=77, F1 `[7,7]` | vector-positive | 0.746688 | 6/6 | -5.40360 | +2.18231 |

The positive t=73 action has a very small full-state normalizer and suppresses
the label's existence, yet its paired three-page branch improves E-OSPA,
RMSE and consistency.  Therefore neither positive advertised source risk nor
large `eta` is a tracking-value certificate; low overlap may represent a
useful suppression as well as a harmful conflict.  By contrast, the frozen
99% two-dimensional chi-square support rule (`d^2 <= 9.21034`, equivalently
the isotropic synopsis compatibility is at least about `0.01`) preserves both
known positive actions and rejects the known t=76 negative action.  It also
rejects the currently observed M24 proposals whose position compatibility is
numerically zero.  The threshold follows the two-dimensional support model
and was not fitted to these three outcomes.

V217 therefore uses position support only as a deterministic semantic guard
against clearly disjoint same-label hypotheses.  It is not promoted as a
tracking guarantee.  Every support-safe, credit-feasible candidate remains a
possibly positive or negative action and must compete with no-op under the
H=3 vector value model.  The next implementation change is to export minimum
receiver position support in the multi-candidate bank, apply this guard before
payload resolution, and label the remaining mode-diverse alternatives rather
than executing the single largest analytic risk score.

That bounded label set is now complete for the first M24 state.  At
`seed 1301 / t=119 / F2`, the live bank contains `39` candidates, of which
`31` pass position support and eight survive mode diversity and exact credit.
All eight H=3 actions are negative under the frozen vector risk budget.  The
best incremental repair, candidate 26, improves E-OSPA/RMSE/consistency by
only `0.012% / 0.240% / 0.154%` relative to withholding-only; relative to the
full-payload reference it still loses `19.267%` mean RMSE, `74.727%` F2 RMSE
and `2.081%` terminal consistency while saving only `0.039%` attempted bytes.
The only candidate retaining the `1%` byte reserve degrades all four aggregate
tracking and consistency coordinates.  This is not a label-selection failure
that warrants another geometric threshold.  It is a no-op preference: the F2
withholding action itself is unsafe and one supported-label edit cannot rescue
it.

The controller must therefore compare no-op, withholding-only and
withholding-plus-label as complete alternatives.  Feasible repair credit does
not obligate the controller to spend it, and a repair head cannot retroactively
certify a harmful withholding head.  The eight negative rows are useful
training evidence, but they are not enough to fit a value model; the next data
step must add independently collected positive and negative states across M24
and X36 before architecture selection or conformal calibration.

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
