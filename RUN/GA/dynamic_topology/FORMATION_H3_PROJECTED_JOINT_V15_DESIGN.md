# Formation H=3 projected-joint action probe v15

## Motivation

The v14 seed-211 timing probe selected difficult states, but its two strict
oracle gains were only `+0.517%` and `0`.  A lightweight source-advantage
diagnostic also failed to rank the known strict gains reliably.  This weakens
the hypothesis that a better scalar event score alone can rescue the current
method.

The v13 teacher action bank may itself be the bottleneck.  The truth-free
posterior-risk projector exactly enumerates all four trust/source modes across
the four M24 formations, applies global disagreement, formation-tail, and
payload constraints, and selects one coordinated mode vector.  The teacher
then discards that joint action and retains only single-formation changes and
trust-0.30 pairs.  v15 probes the omitted coordinated action directly.

## Frozen opened probe

- preset / seed: `m24-formation-fov / 211`;
- already-opened times: `60`, `72`, `104`, `124`;
- actions per state: fixed reference and the exact projected joint action;
- return: projected action for one step, then fixed reference for two steps;
- targets and feasibility: the unchanged six-target strict rule;
- cache source: v14 commit
  `c9c6d4dcdc7ad1cb04fb88a22823e99c7fc5bc53`;
- v15 code records both cache-generation and probe-generation commits.

The four times are outcome-inspected training states.  They may establish only
whether coordinated modes add missing action-space headroom.  They cannot be
used as validation or a generalization result.

## Decision

- If the projected joint action obtains materially stronger strict H=3 gains
  than the singleton/pair bank, the next protocol will freeze a compact
  coordinated proposal bank and test it on unopened seed 223/227 states.
- If it fails, neither event-score tuning nor simply restoring the projector's
  chosen joint action is sufficient.  The method must learn interaction value
  or short-horizon risk while retaining exact topology and reference-fallback
  constraints.

X36 and final seeds remain unopened by this probe.

## Result

The projected joint action produces strict gains `[0, 0, 0, 0]%` on times
`[60, 72, 104, 124]`.  At 60, 72, and 124 the exact projector selects the
all-reference mode vector `[1,1,1,1]`, so it discards known weak H=3 actions
already present in the singleton/pair bank.  At 104 it selects modes
`[3,1,1,1]`, but the six targets are
`[-1.627, -5.718, -1.614, -2.835, +0.737, +0.758]%`: communication improves
while every estimation and consensus target regresses.

The simple action-space repair is therefore falsified.  Exact enumeration and
one-round safety do not imply multi-step value; the current posterior-risk
objective can either collapse to reference or choose the wrong formation.
The next opened probe will test whether the one-step intervention duration is
itself suppressing meaningful M24 headroom before a sequence-value model is
designed.
