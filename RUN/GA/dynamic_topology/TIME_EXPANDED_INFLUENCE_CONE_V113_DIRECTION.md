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
