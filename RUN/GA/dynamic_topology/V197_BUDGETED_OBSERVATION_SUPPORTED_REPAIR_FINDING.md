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

## Long-horizon closure

The frozen X36 seed-211 t=72 controller was then extended from H=3 to H=8
without changing its action budget, risk ranking or cooldown.  V99 alone
improves E-OSPA by `+5.457%`, window consensus by `+8.803%` and attempted
bytes by `+5.423%`, but degrades mean RMSE by `-3.677%`.  V197 releases
`{F2, empty, empty, F3, empty, empty, F5, empty}`.  It recovers only
`+0.484%` RMSE relative to V99 while worsening E-OSPA by `-0.254%`, consensus
by `-1.257%`, and adding `153,344 B`.  Relative to static, the final H=8
result is therefore still `-3.174%` RMSE with `+5.217%` E-OSPA,
`+7.657%` consensus and `+4.886%` byte saving.

The per-formation delta against V99 localizes the failure.  F2 gains about
`8.04` RMSE units and remains the useful first repair; F3 loses about `6.23`
RMSE units, while F5 is RMSE-neutral but worsens E-OSPA.  A causal replay of
the visited posteriors shows that two-page receiver-or-cross-sender support
preserves the first-page F2 candidate but rejects the later F3/F5 releases.
Thus the long-horizon failure is not evidence against a repair token; it is
evidence that cooldown expiry cannot itself authorize spending the token.

## Next decision

Do not tune the set-entry threshold on the opened anchors and do not advance
V197 to fresh seeds.  The minimum next controller should keep the top-one
budget and cooldown but make token spending optional: aggregate label support
over the current and preceding local-posterior pages, including currently
reachable cross-edge senders, and release only if the entered set remains
unsupported over that causal window.  This V198 temporal-abstain mechanism
must first preserve the M24 F4 and X36 F2 H=3 actions; only then should it be
tested recursively on X36 H=8.
