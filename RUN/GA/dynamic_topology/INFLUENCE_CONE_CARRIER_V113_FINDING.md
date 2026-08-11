# V113 finding: fixed carrier direction moves rather than removes boundary harm

## Paired result

All three arms start from the same X36 seed-211 t=72 posterior and H=8
random streams.  The counter-clockwise reference is reused; both clockwise
arms preserve the same physical-edge count, dominant/residual fusion-weight
multiset and rolling-B3 guarantee.

| Arm | Mean E-OSPA | Gain vs CCW | Gain vs mean-best full | Byte change vs CCW | B3 |
|:--|--:|--:|--:|--:|:--:|
| counter-clockwise full | 84.037151 | -- | -- | -- | pass |
| clockwise full | 81.803484 | +2.658% | -- | -0.997% | pass |
| clockwise F2--F5 abstention | 78.479689 | +6.613% | +4.063% | +2.366% | pass |

Clockwise full payload is the lower-mean static baseline, but it does not
uniformly dominate counter-clockwise full payload: its minimum formation gain
relative to counter-clockwise is `-3.119%`.  Static orientation is therefore
already a multi-objective routing decision rather than a neutral convention.

## Strong-gate diagnosis

Relative to the lower-mean clockwise full-payload baseline, the interaction
candidate has:

| Metric | Result | Gate |
|:--|--:|:--:|
| mean E-OSPA gain | +4.063% | fail: below +5% |
| mature-page minimum | +2.588% | fail: below +5% |
| worst-sensor gain | +6.850% | pass |
| minimum formation gain | -1.495% | fail |
| F6 non-gateway terminal gain | -7.206% | fail |
| window / terminal consensus gain | +11.699% / +17.191% | pass |
| attempted-byte saving | +3.330% | pass |
| rolling B3 | pass | pass |

The formation gains are approximately
`[0.000, 0.979, 10.620, 4.576, 9.913, -1.495]%`.  F1 is repaired, but the
delayed loss moves to F6.  The per-page gain remains positive
`[0.909, 3.021, 4.154, 2.960, 2.588, 7.534, 5.941, 5.781]%`, so the failure
is not a collapse of the aggregate mechanism.  It is a localized downstream
boundary failure.

## Causal conclusion

Carrier direction and posterior participation interact constructively:
clockwise abstention reaches a lower absolute E-OSPA than both the clockwise
full baseline and the earlier counter-clockwise V110 action.  However, a
whole-cycle direction switch only chooses which unprotected formation is
first to consume the recursively altered posterior.  It cannot protect both
ends of the time-expanded influence cone.

This closes binary clockwise/counter-clockwise orientation as the policy
output.  It also corrects the X36 baseline contract: future dynamic methods
must compare against the registered static-direction set, including the
lower-mean clockwise arm, rather than an arbitrary counter-clockwise row.

## Method decision

The next bounded oracle must keep the useful clockwise F2--F5 action while
exposing a receiver-level boundary control at F6.  The smallest useful bank
should compare:

1. the two full-payload static directions;
2. clockwise F2--F5 abstention as the mechanism reference;
3. receiver-selective shielding of the altered incoming carrier at F6; and
4. label-complete gradual entry or an alternative safe source for F6.

The action must preserve physicality, the matched message/weight budget and
rolling B3, and must fall back to the better admissible static row when no
receiver-level action is safe.  Only an arm with at least five-percent mean
and mature-page gain and nonnegative formation/F6-tail results authorizes
training the temporal residual model.  Further duration sweeps and binary
cycle-direction sweeps are closed.

V113 is opened, outcome-selected development evidence.  It establishes
action-space structure, not deployability, validation or generalization.
