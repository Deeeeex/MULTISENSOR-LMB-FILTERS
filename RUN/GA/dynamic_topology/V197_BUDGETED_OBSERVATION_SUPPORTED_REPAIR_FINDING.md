# V197 budgeted observation-supported repair finding

## Result

The online V197 controller exactly reproduces the frozen single-release
teacher sequences without receiving formation identifiers as policy features,
target truth, future measurements or future outcomes:

- M24: requested `{[1 3 4], [1 2 3], [1 2 3]}` and released
  `{F4, empty, empty}`;
- X36: requested `{[1 2 4 5], [1 3 4 5], [1 3 4 5]}` and released
  `{F2, empty, empty}`.

The release is chosen by the largest current observation-unsupported
set-entry risk.  The two following pages are protected by the causal repair
cooldown.

| Scale | E-OSPA gain | RMSE gain | Consensus gain | Attempted-byte saving |
|:--|--:|--:|--:|--:|
| M24 | +7.521% | +10.620% | +17.429% | +3.562% |
| X36 | +2.181% | +0.589% | +2.648% | +5.397% |

The M24 worst-sensor gains are `+26.255%` E-OSPA and `0%` RMSE; its minimum
formation gains are `0%` E-OSPA and `-0.145%` RMSE.  The X36 worst-sensor
gains are `0%` E-OSPA and `+7.048%` RMSE; its minimum formation gains are
`0%` E-OSPA and `-1.489%` RMSE.

## Interpretation

V197 is the first deployable rule in this release line that is positive on
all four aggregate axes at both M24 and X36 while matching the independently
identified useful repair actions.  Its key contribution is not a new payload
type: it recognizes that full-posterior repairs are non-additive recursive
actions and explicitly budgets them.

It does not replace the current comprehensive best V187.  X36 aggregate
improvements remain modest and two formations retain small negative RMSE
tails, so the strict development gate is not met.  The main document should
record V197 as a key mechanism advance while keeping the current-best table on
V187.

## Next decision

Do not tune the set-entry threshold on the opened anchors.  The next useful
test is whether V197's top-one token remains positive over a longer horizon
and fresh seeds.  If the X36 RMSE tail persists, the repair ranking should add
a formation-tail term or a joint one-step counterfactual; the action budget
and cooldown remain frozen.

