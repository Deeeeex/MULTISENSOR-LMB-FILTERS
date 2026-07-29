# M24 paired H=3 privileged joint-action return audit

- Generated: 2026-07-29 12:55:02
- Contract: `rolling-safe-privileged-joint-action-return-audit-m24-v1`
- Proposal role: `privileged`
- Return source commits: `39f2be0d8820ba88bcadc211fbdc0b4f2e563ff5`
- Raw return shard-set SHA-256: `11e1076dc51a801476d06ad4381f615d41c350c36e2e5b0b9246846ab749a8bf`
- Complete official grid: `1`
- Seeds: `[7 11 17 19 23 29]`
- Times: `[75 76 77]`
- Reference mean E-OSPA: `15.930822`
- Candidate-oracle mean E-OSPA: `15.022997`
- Candidate-oracle aggregate gain: `5.699%`
- Candidate-oracle attempted-byte deviation: `0.094%`
- All oracle tail nondegrading: `1`
- All oracle hard-safety gates passed: `1`
- States with an admissible >=5% action: `6 / 18`
- Candidate-oracle return gate passed: `0`
- Repair-monitored sensitivity aggregate gain: `8.340%`
- Repair-monitored sensitivity attempted-byte deviation: `0.001%`
- Repair-monitored sensitivity states with >=5% action: `10 / 18`
- Repair-monitored sensitivity return gate passed: `1`
- Privileged full-space top-K capture evaluated: `0`
- Complete proposal gate passed: `0`
- Privileged target return gate passed: `0`
- Value-filtered proposal distillation authorized: `0`
- Candidate returns support critic training: `0`
- Critic training authorized: `0`
- Evidence boundary: This candidate oracle is confined to 70 frozen offline truth-labelled proposal graphs on the 18-state M24 development grid. Passing authorizes value-filtered proposal distillation only; no deployable proposal model or critic has been trained. Top-K capture, unseen-seed M24, X36, metadata traffic and communication-saving claims remain unevaluated.

## State-matched candidate oracle

| Seed | Time | Candidates | Hard feasible | >=5% | Reference E-OSPA | Oracle E-OSPA | Gain | Oracle index/code | Byte deviation | Tail pass | Repair-monitored gain/index/code |
|--:|--:|--:|--:|--:|--:|--:|--:|:--|--:|--:|:--|
| 7 | 75 | 4 | 3 | 0 | 22.344909 | 21.696583 | 2.901% | `2 / 90` | 0.004% | 1 | `2.901% / 2 / 90` |
| 7 | 76 | 4 | 4 | 4 | 22.958518 | 20.653272 | 10.041% | `3 / 91` | 0.098% | 1 | `10.041% / 3 / 91` |
| 7 | 77 | 4 | 4 | 3 | 21.334282 | 19.558266 | 8.325% | `4 / 92` | 0.658% | 1 | `8.325% / 4 / 92` |
| 11 | 75 | 4 | 1 | 1 | 17.432052 | 14.981520 | 14.058% | `2 / 90` | 0.473% | 1 | `15.892% / 3 / 91` |
| 11 | 76 | 4 | 4 | 4 | 19.105302 | 14.759723 | 22.745% | `1 / 00` | 0.675% | 1 | `22.745% / 1 / 00` |
| 11 | 77 | 4 | 0 | 0 | 17.278250 | 17.278250 | 0.000% | `0 / 24` | 0.000% | 1 | `10.436% / 2 / 90` |
| 17 | 75 | 4 | 0 | 0 | 19.670612 | 19.670612 | 0.000% | `0 / 24` | 0.000% | 1 | `16.437% / 3 / 91` |
| 17 | 76 | 4 | 0 | 0 | 16.527002 | 16.527002 | 0.000% | `0 / 24` | 0.000% | 1 | `0.000% / 0 / 24` |
| 17 | 77 | 4 | 3 | 3 | 18.388913 | 16.663111 | 9.385% | `1 / 00` | 0.606% | 1 | `9.385% / 1 / 00` |
| 19 | 75 | 4 | 0 | 0 | 21.721865 | 21.721865 | 0.000% | `0 / 24` | 0.000% | 1 | `0.000% / 0 / 24` |
| 19 | 76 | 4 | 3 | 0 | 21.257311 | 21.257311 | 0.000% | `0 / 24` | 0.000% | 1 | `0.000% / 0 / 24` |
| 19 | 77 | 4 | 0 | 0 | 22.405077 | 22.405077 | 0.000% | `0 / 24` | 0.000% | 1 | `0.000% / 0 / 24` |
| 23 | 75 | 4 | 0 | 0 | 7.028001 | 7.028001 | 0.000% | `0 / 24` | 0.000% | 1 | `11.822% / 3 / 91` |
| 23 | 76 | 4 | 0 | 0 | 7.962549 | 7.962549 | 0.000% | `0 / 24` | 0.000% | 1 | `0.000% / 0 / 24` |
| 23 | 77 | 4 | 0 | 0 | 8.468206 | 8.468206 | 0.000% | `0 / 24` | 0.000% | 1 | `0.000% / 0 / 24` |
| 29 | 75 | 3 | 0 | 0 | 9.200720 | 9.200720 | 0.000% | `0 / 24` | 0.000% | 1 | `15.069% / 1 / 00` |
| 29 | 76 | 4 | 4 | 4 | 8.588788 | 5.499438 | 35.970% | `4 / 92` | 0.731% | 1 | `35.970% / 4 / 92` |
| 29 | 77 | 3 | 0 | 0 | 5.082441 | 5.082441 | 0.000% | `0 / 24` | 0.000% | 1 | `0.000% / 0 / 24` |

## Frozen gates

- Minimum candidate-oracle aggregate gain: `7.0%`
- Maximum attempted-byte deviation: `2.0%`
- Tail nondegradation required: `1`
- Maximum additional repairs versus reference: `0`
