# V120: time-expanded whole-carrier orientation switch

## Question

V116--V119 reject every local repair around the delayed F5-to-F6 boundary:
which labels enter, which F5 sensor sends, which F6 sensor receives and whether
a second formation contributes for one page.  V120 asks a different question:
does the complete direction of recursive information flow need to change once
the protected posterior has matured?

The experiment retains the V113 F2--F5 abstention schedule and the same
60-message, row-stochastic carrier at every page.  Only the orientation of the
six-formation ring may change.  The switch is frozen after page 4, before the
opened V113 trajectory first shows delayed F6 loss.

## Minimal causal comparison

Four paired arms are required:

| Arm | Pages 1--4 | Pages 5--8 | Purpose |
|:--|:--|:--|:--|
| Fixed CCW abstention | CCW | CCW | orientation control; reuse V110 |
| Fixed CW abstention | CW | CW | primary mechanism baseline; reuse V113 |
| CW to CCW | CW | CCW | target time-aligned switch |
| CCW to CW | CCW | CW | reciprocal switch/churn control |

The registered CCW-full and CW-full outcomes remain the no-abstention static
baselines.  Reusing V110 and V113 is valid because their cache, measurements,
delivery uniforms, filter RNG, receiver semantics, route weights and F2--F5
schedule are identical to the corresponding fixed V120 arms.  V120 therefore
runs only the two unknown switch sequences.

## Admission and decision gate

Every page must preserve physicality, 60 selected messages, row stochasticity,
the carrier weight multiset and rolling sensor/formation B3.  The target CW to
CCW arm passes only if it simultaneously:

- improves mean E-OSPA by at least 5% over the better full-payload direction;
- beats both fixed abstention directions and the reciprocal switch in mean
  E-OSPA;
- retains at least 5% gain on every mature page;
- has nonnegative weakest-formation window and terminal gains, worst-sensor
  gain, window/terminal consensus gain and attempted-byte saving;
- satisfies rolling sensor and formation B3 at every page.

A pass establishes formation-level direction-and-timing headroom.  It would
authorize work on an observable carrier-orientation value model.  Failure
closes the current binary whole-carrier direction family: neither a local GNN
nor a switch-time predictor would then have a positive target on this anchor,
and the next method decision must change the carrier action representation.

V120 uses opened X36 seed-211 t=72 H=8 outcomes to choose the switch time.  It
is privileged development evidence, not validation or generalization.
