# Formation H=3 duration probe v16

## Question

The v13/v14 one-step actions have at most weak M24 headroom, while v15 shows
that the exact one-round joint projector either collapses to reference or
misranks the H=3 return.  This probe asks whether one intervention step is too
short for a routing change to affect a 24-node recursive filter materially.

## Frozen opened probe

The probe reuses three outcome-inspected seed-211 states and their known
one-step strict-oracle actions:

| Time | Frozen action | One-step strict gain |
|--:|:--|--:|
| 60 | formations 1-2, trust 0.30 | +1.591% |
| 72 | formation 2, trust 0.70 | +0.024% |
| 104 | formation 4, trust 0.50 | +0.517% |

For the candidate arm, the current-time adjacency and fusion weights are held
unchanged for all three H=3 steps.  They are not recomputed from future
posteriors, measurements, or link outcomes.  Each step must remain physically
valid, avoid repair/emergency/infeasibility, and pass rolling-B3 checks.  The
reference arm keeps the original protocol: its current reference action is
executed at step one and the registered reference is recomputed at steps two
and three.

The six strict targets remain unchanged.  This is a privileged training
mechanism probe, not a deployable duration selector.

## Decision

- Strong, jointly safe gains would justify a causal duration/termination model
  with exact per-step topology safety and reference fallback.
- Tracking gain accompanied by formation, consensus, or byte regression would
  show why blind persistence is unsafe and motivate learned stopping rather
  than a fixed dwell time.
- No gain would rule out intervention duration as the main missing mechanism.

Seeds 223, 227, X36, and all final seeds remain unopened.
