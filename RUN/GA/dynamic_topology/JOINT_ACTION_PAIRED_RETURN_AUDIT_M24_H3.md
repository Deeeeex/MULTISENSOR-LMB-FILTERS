# M24 paired H=3 joint-action return audit

- Generated: 2026-07-29 11:49:29
- Contract: `rolling-safe-joint-action-return-audit-v1`
- Return source commits: `25a3104d908cbd66d21482924a485d7e6c115d8f, 3e90076d66591a76b99df98c00d3a31c45b4c901, fb923c613b30b966efce2876a5ecdd34cae59135`
- Raw return shard-set SHA-256: `46c9125ae5874d03aa6c9a9e06f04a7c44288b1b9208d945c3a7838c7b616a31`
- Complete official grid: `1`
- Seeds: `[7 11 17 19 23 29]`
- Times: `[75 76 77]`
- Reference mean E-OSPA: `15.930822`
- Candidate-oracle mean E-OSPA: `15.230064`
- Candidate-oracle aggregate gain: `4.399%`
- Candidate-oracle attempted-byte deviation: `0.341%`
- All oracle tail nondegrading: `1`
- All oracle hard-safety gates passed: `1`
- States with an admissible >=5% action: `5 / 18`
- Candidate-oracle return gate passed: `0`
- Repair-monitored sensitivity aggregate gain: `5.724%`
- Repair-monitored sensitivity attempted-byte deviation: `0.302%`
- Repair-monitored sensitivity states with >=5% action: `7 / 18`
- Repair-monitored sensitivity return gate passed: `0`
- Privileged full-space top-K capture evaluated: `0`
- Complete proposal gate passed: `0`
- Candidate returns support critic training: `0`
- Critic training authorized: `0`
- Evidence boundary: The candidate-oracle result is confined to the frozen truth-free proposal bank and paired H=3 M24 state grid. The separate privileged full-action-space top-K capture denominator has not been evaluated, so this audit cannot by itself pass the complete proposal gate. Critic, unseen-seed M24, X36, coordinator metadata traffic, and end-to-end communication claims remain unevaluated.

## State-matched candidate oracle

| Seed | Time | Candidates | Hard feasible | >=5% | Reference E-OSPA | Oracle E-OSPA | Gain | Oracle index/code | Byte deviation | Tail pass | Repair-monitored gain/index/code |
|--:|--:|--:|--:|--:|--:|--:|--:|:--|--:|--:|:--|
| 7 | 75 | 21 | 1 | 0 | 22.344909 | 22.344909 | 0.000% | `0 / 24` | 0.000% | 1 | `0.000% / 0 / 24` |
| 7 | 76 | 19 | 10 | 5 | 22.958518 | 20.843121 | 9.214% | `16 / 77` | 1.346% | 1 | `9.214% / 16 / 77` |
| 7 | 77 | 16 | 0 | 0 | 21.334282 | 21.334282 | 0.000% | `0 / 24` | 0.000% | 1 | `0.000% / 0 / 24` |
| 11 | 75 | 23 | 8 | 0 | 17.432052 | 17.416163 | 0.091% | `6 / 64` | 0.250% | 1 | `9.869% / 5 / 63` |
| 11 | 76 | 23 | 13 | 11 | 19.105302 | 13.373478 | 30.001% | `19 / 77` | 1.449% | 1 | `30.001% / 19 / 77` |
| 11 | 77 | 16 | 1 | 0 | 17.278250 | 17.278250 | 0.000% | `0 / 24` | 0.000% | 1 | `0.000% / 0 / 24` |
| 17 | 75 | 16 | 3 | 0 | 19.670612 | 19.030450 | 3.254% | `12 / 74` | 0.433% | 1 | `3.254% / 12 / 74` |
| 17 | 76 | 22 | 10 | 3 | 16.527002 | 15.200104 | 8.029% | `18 / 77` | 1.278% | 1 | `8.029% / 18 / 77` |
| 17 | 77 | 11 | 4 | 0 | 18.388913 | 17.887063 | 2.729% | `6 / 73` | 0.316% | 1 | `2.729% / 6 / 73` |
| 19 | 75 | 17 | 3 | 0 | 21.721865 | 21.721865 | 0.000% | `0 / 24` | 0.000% | 1 | `1.010% / 13 / 78` |
| 19 | 76 | 17 | 16 | 0 | 21.257311 | 20.948979 | 1.450% | `14 / 75` | 0.054% | 1 | `1.450% / 14 / 75` |
| 19 | 77 | 20 | 1 | 0 | 22.405077 | 22.405077 | 0.000% | `0 / 24` | 0.000% | 1 | `0.000% / 0 / 24` |
| 23 | 75 | 20 | 1 | 0 | 7.028001 | 7.028001 | 0.000% | `0 / 24` | 0.000% | 1 | `2.191% / 8 / 67` |
| 23 | 76 | 23 | 9 | 2 | 7.962549 | 7.029902 | 11.713% | `9 / 67` | 1.568% | 1 | `11.713% / 9 / 67` |
| 23 | 77 | 17 | 3 | 0 | 8.468206 | 8.468206 | 0.000% | `0 / 24` | 0.000% | 1 | `0.000% / 0 / 24` |
| 29 | 75 | 21 | 2 | 0 | 9.200720 | 9.200720 | 0.000% | `0 / 24` | 0.000% | 1 | `18.733% / 13 / 71` |
| 29 | 76 | 21 | 20 | 3 | 8.588788 | 7.548138 | 12.116% | `10 / 68` | 0.381% | 1 | `12.116% / 10 / 68` |
| 29 | 77 | 15 | 1 | 0 | 5.082441 | 5.082441 | 0.000% | `0 / 24` | 0.000% | 1 | `0.000% / 0 / 24` |

## Frozen gates

- Minimum candidate-oracle aggregate gain: `7.0%`
- Maximum attempted-byte deviation: `2.0%`
- Tail nondegradation required: `1`
- Maximum additional repairs versus reference: `0`
