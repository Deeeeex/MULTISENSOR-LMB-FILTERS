# M24 fixed-duration H=3 probe audit

## Decision

Persisting an already positive topology action for three steps does not create
additional M24 headroom.  Across three outcome-inspected seed-211 states, the
strict gains change from `[1.591, 0.024, 0.517]%` under the one-step protocol to
`[1.105, 0, 0]%` under a fixed three-step duration.

## Provenance

- Probe generation commit: `4c38bb6079936dbdb889952eff98861ffc8689a0`
- Cache generation commit: `c9c6d4dcdc7ad1cb04fb88a22823e99c7fc5bc53`
- Preset / seed: `m24-formation-fov / 211`
- Times: `60`, `72`, `104`
- Candidate: frozen current adjacency and fusion weights for all three steps
- Reference: original action at step one, recomputed registered reference at
  steps two and three
- Feasibility: the unchanged six-target non-regression rule plus topology
  runtime checks
- Evidence split: privileged training mechanism probe only

| Time | Frozen action | One-step gain | Three-step six targets | Strict result |
|--:|:--|--:|:--|:--|
| 60 | formations 1-2, trust 0.30 | +1.591% | `[+1.105, 0, +0.104, +3.762, +1.000, +1.060]` | +1.105% |
| 72 | formation 2, trust 0.70 | +0.024% | `[+0.105, 0, +0.101, -0.199, +1.034, +0.200]` | reference |
| 104 | formation 4, trust 0.50 | +0.517% | `[+0.630, 0, 0, +1.279, -0.152, -0.156]` | reference |

The time-60 action remains jointly feasible, but its gain falls by `0.486`
percentage points.  The time-72 action becomes infeasible because consensus
regresses, while the time-104 action becomes infeasible because both byte
targets regress.  Persistence therefore changes the kind of accumulated risk;
it does not simply amplify the one-step tracking response.

## Consequence

This probe falsifies fixed dwell time as the main missing mechanism.  It does
not falsify adaptive stopping or sequential topology control, because a policy
could terminate or switch actions as the risk state evolves.  Before building
such a model, the next opened-data audit must determine whether the exact
samplewise zero-regression rule is masking useful risk-controlled actions or
whether the present singleton/pair action bank truly lacks enough return.

No result here is deployable or generalizable: all three actions were selected
after observing their one-step outcomes.  Seeds 223, 227, X36, and all final
seeds remain unopened by this audit.
