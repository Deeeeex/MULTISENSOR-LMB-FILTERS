# V89 repeated multi-gateway handover coverage finding

## Full-axis source result

The current-only scan covered every cached braided-handover state from t=40
to t=140 in steps of four.  It did not execute KLA, route changes or tracking.

| Scale | Active times | Multi-gateway times | Maximum simultaneous gateways | Distinct receiver formations | Mean active formation coverage | Total gateway nominations |
|:--|--:|--:|--:|:--|--:|--:|
| M24 | 26/26 | 15 | 2 | `[2, 3]` | 39.4% | 41 |
| X36 | 26/26 | 26 | 4 | `[2, 3, 4, 5]` | 53.8% | 84 |

Both scales pass the frozen coverage gate: more than 10% of times are
actionable, at least one time has multiple non-conflicting gateways, and the
union covers at least half of the formations.

## Interpretation

The V84 one-edge selector was sparse because it required an exact
mixture-aware KLA existence-net sign before allowing an action.  V88 showed
that this sign is not a valid tracking-value sign.  Once nomination is
restricted to the more defensible question—whether a physical third
formation currently owns unique detection-associated support—the action
space contains broad parallel coverage at both scales.

The 100% active-time fraction does not prove that every nomination is useful.
It shows that unique information supply persists over intervals rather than
appearing only at isolated handover instants.  Consequently the executable
policy must impose temporal structure instead of modifying the graph every
round.

## Frozen implementation direction

The full-episode V89 controller uses a causal three-phase cadence:

1. **Acquire:** select at most one nominated gateway per receiver formation
   and replace its existing `0.05` cross-formation residual sender.
2. **Broadcast:** on the next round, promote each acquired gateway into the
   `0.70` local slot of the other five formation members, retaining each
   displaced dominant sender at `0.05`.
3. **Reference:** return every row to the current physical-tree reference for
   one recovery round.

The phase then repeats with fresh current posterior nominations.  Candidate
sets are projected jointly through physicality, exact per-row message and
weight-multiset parity, at most one gateway per formation, and rolling-B3.
If any invariant fails, the current round uses the reference.

This result authorizes implementation of the paired full-episode development
arm.  It does not authorize a tracking, validation, generalization or
model-training claim.
