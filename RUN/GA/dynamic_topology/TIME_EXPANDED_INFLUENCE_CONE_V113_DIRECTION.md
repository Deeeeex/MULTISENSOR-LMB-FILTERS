# V113 direction: time-expanded influence-cone control

## Problem statement

The corrected receiver semantics and the matched-static X36 screens change
the method target.  The useful action is not an unconstrained change of the
physical tree.  It is a temporary change to posterior participation on an
otherwise accountable carrier graph.  V109 shows that explicit source
abstention creates the aggregate gain.  V110 shows that a perfect local
formation selector retains more than five percent mean headroom, but a safe
formation can still be harmed after an altered posterior travels through
intermediate formations.  V111 shows that a fixed broadcast cadence does not
remove this downstream debt over a longer horizon.

The decision variable must therefore describe both the immediate fusion
action and its finite-horizon propagation:

```text
action = (receiver, source, label set, participation mode, duration)
state  = current posterior + delivery history + unresolved influence debt
```

Physical adjacency says which messages can be sent.  The time-expanded
effective KLA graph says which posterior changes can influence a later
receiver.  These two graphs must not be conflated.

## Analytic influence layer

For each feasible source--receiver action, first execute a truth-free
one-round counterfactual under the same LMB-KLA implementation used by the
filter.  Record label-wise changes in existence log odds, mixture overlap,
state mean and covariance, together with extraction-threshold margins.  A
message-passing sensitivity operator then transports those signed changes
through the planned fusion rows for a finite horizon.

The resulting influence cone contains every receiver--label--time node that
can inherit a non-negligible change.  Its conservative risk statistics are:

1. the smallest supported existence margin inside the cone;
2. the largest covariance-normalized state displacement;
3. the oldest unresolved intervention reaching each receiver;
4. the number and weight of alternative independent information paths; and
5. the worst predicted loss over formations, not only the network mean.

The earlier V66 use of powers of the fusion matrix remains a useful linear
motivation, but its old X36 numerical evidence used the superseded
`support-renormalized` receiver semantics.  V113 must recompute every feature
and label under the corrected explicit-abstention path; no V64--V66 X36 gain
may be reused as current evidence.

## Data-driven residual model

The analytic operator cannot fully represent mixture-component selection,
MAP extraction and state-dependent recursive fusion.  A temporal GNN is used
only to estimate this residual finite-horizon regret.  It does not directly
emit an unconstrained routing graph.

Inputs are current and causal: per-label Bernoulli and mixture summaries,
one-round counterfactual deltas, association support, FoV opportunity, link
quality, fusion weights, graph distance, intervention age, delivery history
and analytic influence-cone statistics.  Target identity, future
measurements, future delivery realizations and tracking truth are excluded.
Training labels come from paired counterfactual rollouts and include mean
gain, minimum formation gain, minimum mature-page gain and worst downstream
regret.  V112 supplies the first duration labels; later datasets must vary
receiver/source/label actions across M24, X36 and non-radial development
states.

## Deterministic safety projection

The learned score ranks feasible actions.  A deterministic projector remains
responsible for estimator and communication invariants:

- preserve the registered physical carrier and rolling B3 reachability;
- retain at least one credible positive-density path for every supported
  label;
- reject an action whose upper-confidence downstream regret is positive;
- cap unresolved influence-cone age and accumulated debt;
- keep attempted bytes within the matched static full-payload budget; and
- fall back to the static full-payload row when uncertainty is high or no
  safe action exists.

This makes the theoretical contribution the construction and control of the
time-expanded effective KLA influence graph.  The GNN approximates a
well-defined nonlinear residual rather than replacing the fusion theory.

## Next action-space screen

V112 decides whether duration alone contains a strictly safe X36 action.  If
one duration passes, V113 should learn a causal stop/recovery score and test it
on unopened M24/X36 states.  If every duration fails, further dwell-time
enumeration stops.  The next upper-bound bank must expose the downstream
influence cone by combining:

1. formation-safe source abstention;
2. receiver-selective shielding of altered outgoing posteriors;
3. an alternative full-payload source when a physically reachable safe path
   exists; and
4. gradual, label-complete re-entry instead of a whole-source binary switch.

The smallest useful oracle bank changes one of these propagation controls at
a time while preserving the paired static outcome.  Only an action family
that reaches at least five percent mean gain with nonnegative formation,
sensor-tail, consensus, communication and B3 results authorizes model
training.

The first bounded X36 counterfactual should test carrier direction before a
larger route search.  The fixed counter-clockwise reference carries altered
F2 output directly toward the otherwise unprotected F1 formation; V110's F1
loss appears only after this delayed propagation.  A clockwise residual cycle
places the protected-to-unprotected boundary at a different formation.  The
screen therefore needs four conceptually distinct outcomes:

| Payload action | Carrier | Purpose |
|:--|:--|:--|
| full | counter-clockwise | registered primary baseline |
| full | clockwise | conditional alternative-static baseline |
| safe-formation abstention | counter-clockwise | reuse the V110 mechanism arm |
| safe-formation abstention | clockwise | test influence-cone redirection |

The candidate must beat the better full-payload carrier rather than only the
registered counter-clockwise row.  Both directions must keep the same
dominant/residual weight multiset, message count, physical-edge constraints
and rolling B3 reserve.  If the interaction arm repairs F1 while retaining
the aggregate gain, it establishes route-conditioned headroom for the later
influence predictor.  If the full clockwise arm explains the result, the
finding is only a stronger static baseline.  If neither repairs the tail,
binary cycle orientation is too coarse and the next bank must expose
receiver-selective or label-complete propagation controls.

## Validation path

After freezing the predictor and projector, the first deployment gate is a
matched comparison on unseen M24 and X36 states and seeds.  Scene transfer
then covers convoy, linear relay, merge--split and curved corridor presets,
followed by at least one larger sensor-count setting.  Report mean and tail
tracking, cardinality, minimum formation and receiver gains, attempted and
delivered bytes, fallback rate, predicted-risk calibration and oracle regret.

This document defines a method direction, not evidence that V113 has passed
any tracking or generalization gate.

## V113 decision update

The carrier-direction interaction screen is complete.  Clockwise full payload
improves mean E-OSPA from `84.037151` to `81.803484` (`+2.658%`) but worsens
at least one formation, so the two static directions form a multi-objective
baseline set.  Adding the opened F2--F5 abstention schedule on the clockwise
carrier reaches `78.479689`: `+6.613%` versus counter-clockwise full but only
`+4.063%` versus the lower-mean clockwise full baseline.

The interaction improves worst-sensor error, consensus and communication,
but its minimum formation gain is `-1.495%` and its F6 non-gateway terminal
gain is `-7.206%`.  F1 is repaired while the delayed boundary loss moves to
F6.  Binary whole-cycle orientation is therefore closed, along with further
duration-only searches.

The next upper-bound action should hold the clockwise F2--F5 mechanism fixed
and vary only the F6 boundary treatment: receiver-selective shielding,
label-complete gradual entry, or a physically reachable alternative safe
source.  This is the smallest screen that can decide whether the
time-expanded cone can be contained locally before temporal-GNN training.
Future gates compare against both registered static directions and use the
better admissible baseline for each declared objective; an arbitrary single
cycle orientation is no longer an adequate static baseline.

## V114 decision update

The receiver-side F6 boundary screen is also complete.  Blocking the single
clockwise cross-formation edge `27 -> 32` on pages 6--8 gives `+4.212%` mean
gain versus clockwise full; starting on page five gives `+4.259%`.  The early
arm is better, but the difference is only `0.049` percentage points and both
remain below the five-percent mean and mature-page gates.

The intervention is nevertheless diagnostically useful.  The early arm
improves the boundary receiver's terminal error by `+7.329%`, worst-sensor
error by `+11.136%`, and terminal consensus by `+19.586%`.  Yet F6
non-gateway sensors remain `-5.522%` below clockwise full.  A binary shield
can stop part of the incoming debt but cannot provide the useful information
needed to reconstruct the downstream F6 posterior.  This closes boundary
timing and whole-posterior receive/reject as primary action variables.

V115 should therefore expose a label-complete gradual-entry action: keep the
physical carrier, weights and B3 route fixed; transmit the full posterior for
labels whose current receiving-side support is credible; defer only labels
whose counterfactual change has unresolved downstream debt; and fall back to
a physically reachable full-payload source when no label-safe entry exists.
The oracle screen must first show that this finer action family can exceed the
matched clockwise-full five-percent gate without a negative formation or
sensor tail.  Only then is temporal-GNN predictor training justified.

## V115 decision update

The first observable label-wise boundary rules are complete and none passes.
Sender support, sender support-or-high-existence, and receiver need all
produce mean E-OSPA `78.47967--78.47968`, numerically identical to the V113
unshielded result `78.479689`.  Every arm remains at `+4.063%` versus
clockwise full, with `-1.495%` minimum formation and `-7.206%` F6
non-gateway terminal gain.  The rules send different label counts and bytes
but do not change the recursive tracking outcome.

This is stronger than a failed threshold.  Current one-step association
support, existence and receiver need cannot identify the labels that create
delayed F6 regret.  Further threshold sweeps, and training a GNN on these
local proxy targets, are closed.  V116 must first test a privileged delayed
return action space on the same opened anchor.  It should search bounded
complete-label subsets on `27 -> 32`, score their full H=8 downstream return,
and report an oracle frontier for mean gain, F6 tail, bytes and runtime.  A
passing subset authorizes learning a time-expanded label-value residual; no
passing subset closes label-wise boundary control and redirects the method to
an alternative physical source or carrier topology.

## V116 decision update

The privileged truth-ranked boundary-subset screen is complete and none of
the top-5, top-10 or top-15 quotas passes.  The best top-5 arm reaches mean
E-OSPA `78.401889`, or `+4.158%` versus clockwise full.  It improves V113 full
boundary entry by `+0.099%` but remains `-0.106%` below V114 empty boundary
entry.  Its mature-page minimum is `+2.769%`, minimum formation gain is
`-0.921%`, and F6 non-gateway terminal gain is `-5.594%`.  Worst-sensor and
consensus metrics are positive, so the unresolved failure is specifically
the downstream F6 information return.

Increasing the quota to 10 or 15 moves the result mostly back toward V113:
the two arms reach only `+4.085%` and `+4.109%` versus clockwise full and F6
peer losses return to approximately `-7.2%`.  Current-truth alignment is a
stronger selector than the V115 observable proxies, yet it still cannot find
a useful intermediate point between full and empty boundary entry.  Further
label quotas, thresholds, and a GNN trained for this `27 -> 32` label action
are therefore closed.

The captured trajectory also localizes the remaining route variable.  Sensor
27 already has the lowest pages-5--8 mean E-OSPA among the six F5 sources
(`69.029`), while sensor 32 has the highest error among the six F6 receivers
(`85.380`).  V117 should retain sender 27, the F5-to-F6 formation edge and the
cross-edge weight, but move the concrete F6 receiver and restore the internal
residual edge displaced by each alternative gateway.  This tests whether the
failure belongs to the gateway location and its downstream influence cone,
rather than to the incoming labels or their source posterior.

## V117 decision update

The paired F6 entry-return migration screen is complete and all candidates
fail.  The best `27 -> 33 / 34 -> 2` arm reaches mean E-OSPA `78.893034`, only
`+3.558%` versus clockwise full and `-0.527%` versus the original V113 gateway.
Its mature-page minimum is `+2.037%`, minimum formation is `-4.331%`, and the
new gateway's terminal gain is `-17.936%`.  The other two candidates reduce
mean gain to `+3.200%` or `+3.494%` and make the F6 non-gateway tail roughly
`-10%`.

Every corrected arm preserves message count, row sums, nonzero weight
multiset, static strong connectivity and rolling B3.  The regression is thus
caused by the changed recursive posterior path, not an invalid carrier.
Current receiver error is not a viable gateway score, and same-source local
gateway placement is closed.

V118 should hold receiver 32 and the entire formation-level carrier fixed and
exhaust the five alternative F5 senders at the same `0.05` cross weight.  This
is the last single-source local control.  If it fails, the next action must
change formation-level information provenance or use a budget-matched
multi-source entry; no additional local gateway or label model is justified.

## V118 decision update

The paired alternative-source screen is complete and all five candidates
fail.  A literal source replacement disconnected sensor 27 because
`27 -> 32` was its only outward influence.  The admitted screen therefore
uses the same minimum repair in every arm, replacing `25 -> 26` by
`27 -> 26`; all arms retain 60 messages, row sums, the fusion-weight multiset,
static strong connectivity and rolling B3.

Source 28 gives the best mean E-OSPA, `78.426149`, or `+4.129%` versus
clockwise full, but is still `-0.137%` behind V114 and makes the F6 peers
`-7.305%` worse at the terminal page.  Source 30 is the minimum-gate-regret
arm because its F6-peer loss is only `-0.416%`, but its mean gain falls to
`+4.008%` and F6 remains the weakest formation at `-2.130%`.  The five-source
mean range is only `0.183026` E-OSPA.

Sender identity therefore carries a weak tail-risk signal but is not a
sufficient control.  V116--V118 jointly close label, receiver and source
selection on this single F5-to-F6 boundary, so no GNN should be trained on the
local action family.  The next bounded screen must change formation-level
information provenance with an exact-budget, horizon-aware second path and a
donor-only ablation; repeating the one-round generic V95 reallocation would
not answer this delayed-return mechanism.

## V119 decision update

The exact-budget time-expanded dual-path screen is complete and all four
sources fail.  Donor-only outcomes remain within `0.001153` E-OSPA of V113,
whereas every added F6 path makes tracking worse than its own deletion
ablation.  The best compatibility-ranked F1 path reaches `78.501408`, or
`+4.037%` versus clockwise full but `-0.026%` versus donor-only and `-0.233%`
versus V114.  Its F6 formation is `-1.647%`, non-target F6 peers are `-5.217%`,
and the target receiver is `-9.336%` at the terminal page.  Lower-compatibility
F2--F4 sources produce still larger mean regressions.

The result cleanly rejects a local complementary-source explanation: the
removed ring input is neutral, while the new F6 input causes the loss.  KLA
compatibility partly orders harm but finds no positive action.  V116--V119 now
close local label, source, receiver and second-path controls.

V120 should therefore change the complete formation carrier.  It will compare
fixed counter-clockwise abstention, a clockwise-to-counter-clockwise switch
before the F6 loss, and the reciprocal switch against the frozen V113
clockwise-abstention baseline.  Fixed-orientation and reciprocal controls are
required to distinguish direction, timing and generic switching effects.

## V120 decision update

The whole-carrier switch screen is complete and the target sequence fails.
Clockwise-to-counter-clockwise reaches mean E-OSPA `78.581372`, or `+3.939%`
versus the better full-payload direction.  It improves fixed CCW by `+1.178%`
and the reciprocal switch by `+1.138%`, but remains `-0.130%` behind fixed CW.
Its mature-page minimum is `+2.770%`, minimum formation gain is `-1.203%`, and
minimum terminal formation gain is `-5.831%`.

The switch reduces the F6 window loss from `-1.495%` under fixed CW to
`-1.203%`, but creates a new `-1.073%` F1 loss.  The reciprocal sequence ends
within `+0.040%` of fixed CCW, showing that the first four pages determine much
of the later recursive state.  A later global orientation reversal therefore
redistributes ring-tail risk rather than repairing it, and the binary
whole-carrier switch family is closed.

V121 should break the global coupling by constructing exact-budget mixed
carriers whose formation segments may use different influence directions in
the same page.  A structural enumeration must precede tracking and retain
physicality, row stochasticity, the weight multiset, 60 messages and rolling
B3.  If no mixed carrier provides a bounded positive oracle, segment-level
edge learning and a GNN remain unjustified.

## V121 decision update

The exact-budget mixed-carrier screen is complete and all four static
formation orders fail.  Moving F6 after F1, F2, F3 or F4 reaches only
`+2.565%` to `+3.692%` mean gain versus clockwise full, and every arm is
`0.386%` to `1.562%` worse than fixed-clockwise abstention.  All candidates
retain the 60-message budget, weight multiset, physicality, a single sensor
and formation cycle, and rolling B3.

The least-regret F6-after-F4 order improves F1 by `+2.558%`, but makes F6
`-6.439%` worse over the window and `-9.751%` worse at the terminal page.
Static one-cycle permutation therefore separates the earlier F1/F6 direction
coupling only by concentrating the debt in F6.  Segment-level edge learning
over these static permutations is closed.  A time-local complementary carrier
is worth screening only if page-resolved evidence shows a positive F6 response
at the onset of the V113 delayed loss; otherwise the representation must leave
the single-Hamiltonian-cycle family.
