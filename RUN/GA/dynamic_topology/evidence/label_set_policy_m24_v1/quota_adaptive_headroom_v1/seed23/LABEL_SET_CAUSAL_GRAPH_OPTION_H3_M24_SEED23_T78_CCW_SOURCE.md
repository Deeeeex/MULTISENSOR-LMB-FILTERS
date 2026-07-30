# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `23 / 78`
- Audit mode: `quota-adaptive-headroom`
- Return times: `[78 79 80]`
- Source SHA-256: `e7677e4962c65842a50ea196348d8380b1160b7fe925dacfa478f54c45fc5e96`
- Source behavior: `fixed-counter-clockwise`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `LLL`
- Best gain versus CCC: `19.620%`

- Registered projection failures: `1`

| Option | Status | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | completed | 7.341908 | 30.494389 | 0.083333 | 10.138767 | 6055968 | +0.000% | 1 |
| `LLL` | completed | 5.901460 | 13.736025 | 0.027778 | 7.549215 | 6055968 | +0.000% | 1 |
| `Q3-Q3-Q3` | completed | 5.927627 | 13.779280 | 0.027778 | 7.535395 | 6051264 | -0.078% | 1 |
| `Q4-Q4-Q4` | completed | 5.936384 | 13.725583 | 0.027778 | 7.546251 | 6054624 | -0.022% | 1 |
| `Q6-Q6-Q6` | completed | 7.146067 | 22.038983 | 0.083333 | 9.768285 | 6055968 | +0.000% | 1 |
| `Q8-Q8-Q8` | completed | 7.148413 | 21.995459 | 0.083333 | 9.777668 | 6055296 | -0.011% | 1 |
| `Q10-Q10-Q10` | completed | 8.192863 | 22.072069 | 0.125000 | 11.425753 | 6055968 | +0.000% | 1 |
| `Q12-Q12-Q12` | infeasible | Inf | Inf | Inf | Inf | NaN | +NaN% | 0 |

## Hard failures

- `Q12-Q12-Q12`: `RollingMatching:Infeasible` — No feasible joint rolling matching exists (glpk error 10, status -1).

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
