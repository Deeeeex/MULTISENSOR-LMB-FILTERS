# M24 observable score-basis coverage audit: seed 7

- Protocol: `m24-observable-score-basis-coverage-seed7-v1`
- Registry SHA-256: `58a558d0ebb423ca731b7c16ba3d698046785ec3d8340c400509a8dcc30dd7c7`
- Evidence split: `development`
- Design-seen seed: `7`
- Candidate count: `19`
- Audit git commit: `3b76cd69b36076bcd0677f37d27abf3ffd3e6c73`
- Audit tracked worktree dirty: `0`
- Audit untracked source dirty: `0`
- Evidence boundary: action-space coverage only; seed 7 is not held-out validation and cannot support a generalization claim.

## Registered reference and gates

- Code-24 E-OSPA: `22.3449`
- Code-24 worst-node E-OSPA: `34.5380`
- Code-24 attempted bytes: `3472848`
- Headroom gate: a new realized route must provide at least 5% mean gain, no worst-node regression, at most 2% attempted-byte mismatch, no repair or payload emergency, and complete selected rolling-B3 safety.

## Complete candidate table

| Code | Basis | k | E-OSPA | Mean gain | Worst node | Tail gain | Byte change | Repair | New route | Pass |
|--:|:--|--:|--:|--:|--:|--:|--:|--:|:--:|:--:|
| 60 | `link-advantage` | 1 | 24.4974 | -9.63% | 38.6686 | -11.96% | +0.78% | 0.3333 | 1 | 0 |
| 61 | `link-advantage` | 2 | 23.8548 | -6.76% | 38.6686 | -11.96% | +0.56% | 0.3333 | 1 | 0 |
| 62 | `link-advantage` | 3 | 23.8239 | -6.62% | 38.6686 | -11.96% | -0.79% | 0.3333 | 1 | 0 |
| 63 | `posterior-gain` | 1 | 24.5176 | -9.72% | 47.6170 | -37.87% | +0.60% | 0.3333 | 1 | 0 |
| 64 | `posterior-gain` | 2 | 23.3417 | -4.46% | 47.6170 | -37.87% | +0.54% | 0.0000 | 1 | 0 |
| 65 | `posterior-gain` | 3 | 23.3447 | -4.47% | 47.6170 | -37.87% | -0.70% | 0.0000 | 1 | 0 |
| 66 | `compatibility-conservative` | 1 | 23.7106 | -6.11% | 37.8442 | -9.57% | -0.29% | 0.3333 | 1 | 0 |
| 67 | `compatibility-conservative` | 2 | 23.2863 | -4.21% | 38.7907 | -12.31% | -0.27% | 0.3333 | 1 | 0 |
| 68 | `compatibility-conservative` | 3 | 24.5823 | -10.01% | 40.3537 | -16.84% | -0.07% | 0.0000 | 1 | 0 |
| 69 | `receiver-rescue` | 1 | 23.1818 | -3.75% | 38.5774 | -11.70% | +0.54% | 0.0000 | 1 | 0 |
| 70 | `receiver-rescue` | 2 | 23.5809 | -5.53% | 39.3178 | -13.84% | +0.56% | 0.0000 | 1 | 0 |
| 71 | `receiver-rescue` | 3 | 24.0683 | -7.71% | 42.0963 | -21.88% | +0.30% | 0.0000 | 1 | 0 |
| 72 | `history-novelty` | 1 | 24.2297 | -8.43% | 44.4508 | -28.70% | +0.22% | 0.3333 | 1 | 0 |
| 73 | `history-novelty` | 2 | 25.1962 | -12.76% | 44.4508 | -28.70% | -0.08% | 0.3333 | 1 | 0 |
| 74 | `history-novelty` | 3 | 23.3243 | -4.38% | 41.8686 | -21.22% | +0.04% | 0.0000 | 1 | 0 |
| 75 | `history-continuity` | 1 | 24.4974 | -9.63% | 38.6686 | -11.96% | +0.78% | 0.3333 | 1 | 0 |
| 76 | `history-continuity` | 2 | 23.8548 | -6.76% | 38.6686 | -11.96% | +0.56% | 0.3333 | 1 | 0 |
| 77 | `history-continuity` | 3 | 23.8239 | -6.62% | 38.6686 | -11.96% | -0.79% | 0.3333 | 1 | 0 |
| 78 | `no-cross-edge` | 0 | 23.1663 | -3.68% | 41.9307 | -21.40% | +1.26% | 0.3333 | 0 | 0 |

## Diversity and headroom decision

- Unique realized sensor routes: `16 / 19`
- New no-repair sensor routes versus old codebook: `7`
- New no-repair formation routes versus old codebook: `7`
- Covered nonzero exact cardinalities: `[1 2 3]`
- Projector duplicate rate: `15.79%`
- Diversity gate passed: `1`
- Joint headroom pass count: `0`
- Action-space headroom passed: `0`

The registry creates genuine new sensor-level actions, but none improves even the mean E-OSPA over code 24; every candidate also regresses worst-node E-OSPA. The failure is therefore not caused by duplicate projector outputs alone. This local observable linear-score family should be rejected before any selector or value model is trained.

## Source summaries

- `RUN/GA/dynamic_topology/evidence/rollout_dataset/seed7_scorebasis_b01of04/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260729_054219.mat`
- `RUN/GA/dynamic_topology/evidence/rollout_dataset/seed7_scorebasis_b02of04/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260729_054216.mat`
- `RUN/GA/dynamic_topology/evidence/rollout_dataset/seed7_scorebasis_b03of04/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260729_054220.mat`
- `RUN/GA/dynamic_topology/evidence/rollout_dataset/seed7_scorebasis_b04of04/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260729_054051.mat`
