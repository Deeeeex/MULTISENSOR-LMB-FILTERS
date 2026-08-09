# V65: network-additive formation risk with useful-information protection

## Problem corrected

V64 proves a minimal-prefix property for locally normalized formation scores,
but those scores do not add to a physical network-wide fraction when
formations carry different reference existence mass.  V65 uses the same
current one-formation counterfactuals while separating two absolute masses:

- receiver-supported rescue: locally measured existence restored by
  withholding cross-formation fusion;
- cross-supported useful loss: existence currently supported by an active
  cross-formation sender that the same withholding action may remove.

Both are divided by one network reference-existence mass.  Their sums therefore
have the same interpretation on M24 and X36.

## Frozen selector before X36 t=128

The current-only action is enabled only when total additive rescue exceeds 1%
of network reference existence.  It then finds the smallest formation subset
covering at least 80% of that rescue, subject to three guards: every selected
formation has an available current counterfactual, no cross-supported label is
pushed below the 0.5 extraction threshold, and total cross-supported useful
loss does not exceed total receiver-supported rescue.  No feasible set means
deterministic reference fallback.

V65 deliberately does not reuse the older reference-retention ratio as an
admissibility gate.  That ratio protects every label present under the
reference action, including labels with no current cross-sender measurement
support; at X36 t=72 it rejects formations 2 and 4 even though their
cross-supported useful-loss mass is small and no supported label crosses
downward.  Reusing it would reintroduce the stale-label false positives that
the V60 measurement-support correction was designed to remove.

The 1% quantity is an explicit network distortion budget, not a claimed 1%
E-OSPA bound.  The exact covering problem is enumerated for four or six
formations; a larger system can solve the same small binary covering problem
with branch-and-bound or a learned proposal followed by the unchanged safety
projection.

## Data-driven extension boundary

The analytic rule decides whether an intervention is admissible and which
current risk it must cover.  A future GNN may estimate persistence beyond the
current step or non-additive interactions between formations.  It must not
replace the physical-connectivity, useful-information, or reference-fallback
constraints.

## Frozen X36 observable decisions

The committed selector was evaluated on all three registered seed-211 X36
states without opening the t=128 tracking outcome:

| State | Network rescue | Useful loss | Decision | Coverage |
|:--|--:|--:|:--|--:|
| t=72 | `1.512%` | `0.050%` | protect `[2,3,4,5]` | `81.126%` |
| t=100 | `2.267%` | `0.016%` | protect `[2,4,5,6]` | `81.846%` |
| t=128 | `0.428%` | `0.006%` | reference fallback | -- |

The two activated sets are exactly the formation sets whose frozen V64
schedules already produced `+5.847%` and `+9.329%` mean X36 tracking gains.
The low-risk t=128 state is no longer forced into an intervention merely
because a relative top-prefix always exists.
