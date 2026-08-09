# V64 X36 risk-coverage result at t=100

The unchanged 80%-coverage selector chose formations `[6,4,5,2]`, covering
82.9812% of the current observable rescue score.  The protected set remained
fixed over the three-step window while the physical carrier graph and
registered fusion weights stayed unchanged.

| Metric | Candidate relative to reference | Registered requirement |
|:--|--:|:--:|
| Mean tracking | `+9.329%` | at least `+5%` |
| Worst sensor | `+11.723%` | nonnegative |
| Minimum formation | `+0.000%` | nonnegative |
| Window consensus | `+21.467%` | nonnegative |
| Terminal consensus | `+22.966%` | nonnegative |
| Attempted bytes | `+4.533%` | nonnegative |
| Rolling B3 connectivity | pass | pass |

The candidate passes every strict requirement.  Together with the independent
t=72 development result (`+5.847%` mean tracking), this shows that persistent
protection of the dominant observable-risk formations has X36 headroom in two
different network states.  It remains opened seed-211 development evidence;
no held-out, cross-scene, or validation claim is authorized.

## Method implication

The result supports the mechanism but does not remove V64's normalization
limitation.  Its formation scores are locally normalized before aggregation,
so the next rule should use absolute counterfactual mass with one network-wide
denominator and explicitly guard useful cross-supported information before
claiming scale-comparable risk coverage.
