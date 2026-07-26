# Formation-gateway proxy audit

- Generated: 2026-07-26 10:22:18
- Candidate policies: `9`
- Training blocks used for selection: `[1 2]`
- Read-only validation blocks: `3`
- Training gate passed: `0`
- Validation gate passed: `0`
- All observed blocks pass: `0`
- Evidence boundary: Truth-free scores are selected on declared training snapshots; privileged labels only audit selected actions. Validation labels do not affect ranking. No closed-loop or held-out claim.

## Selected training-only policy

- sourceWeight: `0.5`
- baselinePhase: `1`
- positiveExistenceWeight: `1`
- positivePrecisionWeight: `0`
- negativeExistencePenalty: `0`
- negativePrecisionPenalty: `50`
- discrepancyPenalty: `8`
- receiverNeedMultiplier: `0`
- marginThreshold: `0.02`
- minimumPositiveExistence: `0.05`

| Block | Split | Risk improvement vs RR | Worst override | Harmful overrides | Overrides | Formation coverage | Weak connected | Bytes ratio | Oracle capture |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|--:|
| m24-hard seed 7 t=75 | train | 0.0601 | 0.2116 | 0.0000 | 3 | 0.7500 | 0 | 0.9917 | 0.7289 |
| m24-hard seed 11 t=75 | train | 0.0522 | 0.1047 | 0.0000 | 2 | 0.5000 | 0 | 0.9936 | 0.5033 |
| m24-hard seed 17 t=75 | validation | 0.0724 | 0.0330 | 0.0000 | 2 | 0.5000 | 0 | 1.0069 | 0.5864 |

## Top policies by training-only safety key

| Rank | w | phase | +exist | +precision | -exist | -precision | discrepancy | need | margin | min novelty | Worst train gain | Train harmful | Validation gain | Validation harmful |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| 1 | 0.50 | 1 | 1.00 | 0.00 | 0.00 | 50.00 | 8.00 | 0.00 | 0.020 | 0.050 | 0.0522 | 0.0000 | 0.0724 | 0.0000 |
| 2 | 0.50 | 1 | 1.00 | 0.00 | 0.00 | 50.00 | 8.00 | 0.00 | 0.025 | 0.050 | 0.0522 | 0.0000 | 0.0724 | 0.0000 |
| 3 | 0.50 | 1 | 1.00 | 0.00 | 0.00 | 50.00 | 8.00 | 0.00 | 0.030 | 0.050 | 0.0462 | 0.0000 | 0.0724 | 0.0000 |
| 4 | 0.50 | 1 | 1.00 | 0.00 | 0.00 | 50.00 | 8.00 | 0.00 | 0.020 | 0.060 | 0.0439 | 0.0000 | 0.0724 | 0.0000 |
| 5 | 0.50 | 1 | 1.00 | 0.00 | 0.00 | 50.00 | 8.00 | 0.00 | 0.025 | 0.060 | 0.0439 | 0.0000 | 0.0724 | 0.0000 |
| 6 | 0.50 | 1 | 1.00 | 0.00 | 0.00 | 50.00 | 8.00 | 0.00 | 0.030 | 0.060 | 0.0439 | 0.0000 | 0.0724 | 0.0000 |
| 7 | 0.50 | 1 | 1.00 | 0.00 | 0.00 | 50.00 | 8.00 | 0.00 | 0.020 | 0.040 | 0.0140 | 0.3333 | 0.0724 | 0.0000 |
| 8 | 0.50 | 1 | 1.00 | 0.00 | 0.00 | 50.00 | 8.00 | 0.00 | 0.025 | 0.040 | 0.0140 | 0.3333 | 0.0724 | 0.0000 |
| 9 | 0.50 | 1 | 1.00 | 0.00 | 0.00 | 50.00 | 8.00 | 0.00 | 0.030 | 0.040 | 0.0140 | 0.3333 | 0.0724 | 0.0000 |

The round-robin backbone sends one same-formation posterior to every receiver. The candidate keeps the same message count and replaces at most one route per formation. Validation columns are descriptive only and never affect candidate ranking. Harmful means negative teacher residual relative to round-robin, not absolute harm relative to no communication.
