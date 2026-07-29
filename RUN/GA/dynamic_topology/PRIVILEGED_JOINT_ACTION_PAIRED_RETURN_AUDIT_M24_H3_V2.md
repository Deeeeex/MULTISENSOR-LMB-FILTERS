# M24 privileged paired H=3 return audit: v2 semantics

- Generated: 2026-07-29 13:08:35
- Contract: `rolling-safe-privileged-joint-action-return-audit-m24-v2`
- Generation commit: `e9c610b939b5c0665a176f93d60072c3e35f0110`
- Source v1 audit SHA-256: `78fa2c34dc0d1793951227daf3eecf2d0acf39ec53c9096ca7c97bc5f943ff00`
- Raw shard-set SHA-256: `11e1076dc51a801476d06ad4381f615d41c350c36e2e5b0b9246846ab749a8bf`
- Development reanalysis: `1`
- Validation claim allowed: `0`
- First graph exact required: `1`
- First-step repair allowed: `0`
- Later exact projection activation allowed: `1`
- Reference mean E-OSPA: `15.930822`
- Candidate-oracle mean E-OSPA: `14.602248`
- Candidate-oracle aggregate gain: `8.340%`
- Attempted-byte deviation: `0.001%`
- All oracle tail nondegrading: `1`
- All oracle safety gates passed: `1`
- States with an admissible >=5% target: `10 / 18`
- Value-filtered candidate targets: `34`
- Candidate-oracle return gate passed: `1`
- Value-filtered proposal distillation authorized: `1`
- Proposal model training completed: `0`
- Top-K capture evaluated: `0`
- Critic training authorized: `0`
- X36 authorized: `0`
- Evidence boundary: This v2 result reuses design-seen v1 M24 returns after the continuation-repair sensitivity was inspected. It freezes a method rule for future work: the proposed first graph must be exact and repair-free, while later exact safety-projection activation is legal and remains subject to tail, byte, B3, emergency and infeasibility gates. Passing authorizes only value-filtered proposal distillation on development data; it does not validate a proposal model, critic, unseen M24 or X36.

## State-matched v2 oracle

| Seed | Time | Candidates | V2 feasible | >=5% | Reference | Oracle | Gain | Oracle index/code | Byte deviation | Tail pass |
|--:|--:|--:|--:|--:|--:|--:|--:|:--|--:|--:|
| 7 | 75 | 4 | 3 | 0 | 22.344909 | 21.696583 | 2.901% | `2 / 90` | 0.004% | 1 |
| 7 | 76 | 4 | 4 | 4 | 22.958518 | 20.653272 | 10.041% | `3 / 91` | 0.098% | 1 |
| 7 | 77 | 4 | 4 | 3 | 21.334282 | 19.558266 | 8.325% | `4 / 92` | 0.658% | 1 |
| 11 | 75 | 4 | 4 | 4 | 17.432052 | 14.661744 | 15.892% | `3 / 91` | 0.739% | 1 |
| 11 | 76 | 4 | 4 | 4 | 19.105302 | 14.759723 | 22.745% | `1 / 00` | 0.675% | 1 |
| 11 | 77 | 4 | 1 | 1 | 17.278250 | 15.475104 | 10.436% | `2 / 90` | 0.954% | 1 |
| 17 | 75 | 4 | 4 | 4 | 19.670612 | 16.437296 | 16.437% | `3 / 91` | 0.079% | 1 |
| 17 | 76 | 4 | 0 | 0 | 16.527002 | 16.527002 | 0.000% | `0 / 24` | 0.000% | 1 |
| 17 | 77 | 4 | 3 | 3 | 18.388913 | 16.663111 | 9.385% | `1 / 00` | 0.606% | 1 |
| 19 | 75 | 4 | 0 | 0 | 21.721865 | 21.721865 | 0.000% | `0 / 24` | 0.000% | 1 |
| 19 | 76 | 4 | 3 | 0 | 21.257311 | 21.257311 | 0.000% | `0 / 24` | 0.000% | 1 |
| 19 | 77 | 4 | 0 | 0 | 22.405077 | 22.405077 | 0.000% | `0 / 24` | 0.000% | 1 |
| 23 | 75 | 4 | 4 | 4 | 7.028001 | 6.197174 | 11.822% | `3 / 91` | 0.644% | 1 |
| 23 | 76 | 4 | 0 | 0 | 7.962549 | 7.962549 | 0.000% | `0 / 24` | 0.000% | 1 |
| 23 | 77 | 4 | 0 | 0 | 8.468206 | 8.468206 | 0.000% | `0 / 24` | 0.000% | 1 |
| 29 | 75 | 3 | 3 | 3 | 9.200720 | 7.814304 | 15.069% | `1 / 00` | 1.120% | 1 |
| 29 | 76 | 4 | 4 | 4 | 8.588788 | 5.499438 | 35.970% | `4 / 92` | 0.731% | 1 |
| 29 | 77 | 3 | 0 | 0 | 5.082441 | 5.082441 | 0.000% | `0 / 24` | 0.000% | 1 |

## Frozen gates

- Minimum aggregate gain: `7.0%`
- Maximum attempted-byte deviation: `2.0%`
- Tail nondegradation required: `1`
