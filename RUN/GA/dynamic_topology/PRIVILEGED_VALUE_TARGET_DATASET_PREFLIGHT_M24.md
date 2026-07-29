# Privileged value-target dataset preflight: M24

- Generated: 2026-07-29 13:27:44
- Audit contract: `rolling-safe-privileged-value-target-audit-m24-v1`
- Dataset SHA-256: `cfda289bb32ecf35161786476878210b76a0ccdb3b4dfb4ee719b576e66bea63`
- Dataset generation commit: `240be48e1013206f8ad47576756e76c272d69e15`
- Candidate graphs: `70`
- Value-filtered targets: `34`
- Target-bearing states: `10 / 18`
- States collected under privileged behavior: `18 / 18`
- Target counts by action code: `00:9, 90:10, 91:7, 92:8`
- Maximum edge-feature replay difference: `0`
- Maximum joint-feature replay difference: `0`
- Accepted for proposal development: `1`
- Accepted for top-K capture claim: `0`
- Accepted for critic training: `0`
- Accepted for validation/X36: `0 / 0`
- Evidence boundary: PASS establishes an exact, provenance-frozen join between edge features computed without truth/future outcomes and offline truth/future-return labels. The state histories were collected under outcome-selected privileged safe behavior, so even seed-disjoint resampling of these 18 states is method development rather than online generalization. A fresh truth-free rollout distribution must establish top-K capture before critic training, M24 validation or X36 transfer.

## State blocks

| Seed | Time | Candidates | V2 feasible | V2 targets | Value oracle | Oracle gain |
|--:|--:|--:|--:|--:|:--|--:|
| 7 | 75 | 4 | 3 | 0 | `-` | 2.901% |
| 7 | 76 | 4 | 4 | 4 | `91` | 10.041% |
| 7 | 77 | 4 | 4 | 3 | `92` | 8.325% |
| 11 | 75 | 4 | 4 | 4 | `91` | 15.892% |
| 11 | 76 | 4 | 4 | 4 | `00` | 22.745% |
| 11 | 77 | 4 | 1 | 1 | `90` | 10.436% |
| 17 | 75 | 4 | 4 | 4 | `91` | 16.437% |
| 17 | 76 | 4 | 0 | 0 | `-` | 0.000% |
| 17 | 77 | 4 | 3 | 3 | `00` | 9.385% |
| 19 | 75 | 4 | 0 | 0 | `-` | 0.000% |
| 19 | 76 | 4 | 3 | 0 | `-` | 0.000% |
| 19 | 77 | 4 | 0 | 0 | `-` | 0.000% |
| 23 | 75 | 4 | 4 | 4 | `91` | 11.822% |
| 23 | 76 | 4 | 0 | 0 | `-` | 0.000% |
| 23 | 77 | 4 | 0 | 0 | `-` | 0.000% |
| 29 | 75 | 3 | 3 | 3 | `00` | 15.069% |
| 29 | 76 | 4 | 4 | 4 | `92` | 35.970% |
| 29 | 77 | 3 | 0 | 0 | `-` | 0.000% |

## Decision

PASS: all 70 privileged graphs and return labels join exactly; 34 graphs in 10/18 states pass the frozen v2 five-percent value gate. Per-edge feature computation replays without truth/future inputs, but all state histories come from outcome-selected privileged behavior. Multi-solution proposal-head development may proceed; online top-K capture remains unevaluated.
