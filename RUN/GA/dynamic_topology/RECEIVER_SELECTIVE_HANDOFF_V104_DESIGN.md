# V104: receiver-selective matured-handoff oracle

## Minimum-granularity question

V103 proves that formation-wide handoff is too coarse: F2 and F4 contain both
positive and negative peers, all F1 peers regress, and only selected F3/F5
peers improve.  This does not yet prove that label granularity is necessary.
The smallest next question is whether choosing the receiver rows is already
sufficient.

V104 is a retrospective upper-bound diagnostic, not an online policy.  For
each V103 handoff page it retains only changed receiver rows whose paired
same-page E-OSPA gain was strictly positive:

| t | V103 changed receivers | V104 retained receivers |
|--:|:--|:--|
| 75 | 3--6, 9--12, 21--24, 27--30 | 9,10,22,28,29,30 |
| 77 | 15--18 | 17 |
| 79 | 33--36 | 34,36 |

All other rows return to the matched static route.  Because target truth and
the already opened V103 outcome define this set, V104 cannot be presented as
causal, deployable or validation evidence.  It asks only whether a perfect
receiver classifier could remove the V103 collateral damage.

## Frozen execution contract

The V103 protection schedule, three-page maturity rule and H=8 timing remain
unchanged.  A retained receiver copies its exact V103 adjacency and weight
row; every other receiver uses the static row.  Consequently every row keeps
its message count and positive-weight multiset, and all edges remain physical.
The sequence must also pass the same rolling sensor- and formation-level B3
check before tracking outcomes open.

The previously frozen H=8 static outcome is reused rather than rerun.  Reuse
requires an exact match of preset, seed, current time, horizon, receiver mode,
cache path and cache SHA-256.  Only the V104 candidate is executed.  Candidate
pre-fusion posterior snapshots are retained at t=72--79 to support the next
label-level oracle if receiver selection remains insufficient.

## Gate and decision

The strict V103 gate is unchanged: at least 5% mean gain and post-handoff
floor, at least 1% minimum formation gain, at least 1% F6 non-gateway terminal
gain, nonnegative worst-sensor and consensus tails, nonincreasing attempted
bytes, and rolling B3.

- Passing means receiver-level edge-value prediction is sufficient; the next
  learned model should predict receiver handoff value and does not need a
  label action space.
- Failure, despite outcome-informed receiver selection, is evidence that the
  useful and harmful effects coexist within a receiver posterior.  V105 then
  opens the bounded receiver--sender--label oracle described in the task-
  aligned label-value direction.

## Result and attribution correction

V104 lowers mean E-OSPA from 84.037151 to 79.555155, a 5.333% gain, and saves
6.091% attempted bytes.  This is essentially identical to V103: candidate
E-OSPA changes by only +0.000415, minimum formation gain changes from -0.945%
to -0.931%, and F6 non-gateway terminal gain changes from -2.948% to -2.945%.
The strict gate fails.

The result-informed receiver oracle therefore does not remove the local
regressions.  More importantly, F1 still regresses even though V104 changes no
F1 handoff row, and F6 peer 33 still regresses although its handoff row is
restored to reference.  The earlier attribution to whole-posterior handoff was
too strong: the harmful state can arrive recursively from other changed rows,
or be created by extending the control-only protection schedule through H=8.

Do not open label actions yet.  The minimum next experiment is an H=8
protection-only ablation with every topology row fixed to reference.  Only the
difference between that arm and V103/V104 can identify handoff value.  Label
granularity becomes justified only if protection-only is locally safe but the
handoff arms are not.
