# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `19 / 78`
- Audit mode: `quota-adaptive-headroom`
- Return times: `[78 79 80]`
- Source SHA-256: `d1d4a88021ceea4d547668dbfa1c5f31db2d82e245d553c1c5e3d64e2ffa02b9`
- Source behavior: `fixed-counter-clockwise`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `Q3-Q3-Q3`
- Best gain versus CCC: `0.029%`

- Registered projection failures: `2`

| Option | Status | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | completed | 19.856173 | 38.877235 | 0.638889 | 23.436646 | 5966592 | +0.000% | 1 |
| `LLL` | completed | 20.285314 | 40.545148 | 0.680556 | 24.067267 | 5961048 | -0.093% | 1 |
| `Q3-Q3-Q3` | completed | 19.850353 | 40.554201 | 0.652778 | 23.528291 | 5965056 | -0.026% | 1 |
| `Q4-Q4-Q4` | completed | 20.214267 | 40.549787 | 0.666667 | 23.394647 | 5954760 | -0.198% | 1 |
| `Q6-Q6-Q6` | completed | 21.632709 | 40.566509 | 0.736111 | 24.470582 | 5953056 | -0.227% | 1 |
| `Q8-Q8-Q8` | completed | 21.140723 | 42.923724 | 0.708333 | 24.672041 | 5961360 | -0.088% | 1 |
| `Q10-Q10-Q10` | infeasible | Inf | Inf | Inf | Inf | NaN | +NaN% | 0 |
| `Q12-Q12-Q12` | infeasible | Inf | Inf | Inf | Inf | NaN | +NaN% | 0 |

## Hard failures

- `Q10-Q10-Q10`: `RollingMatching:Infeasible` — No feasible joint rolling matching exists (glpk error 10, status -1).
- `Q12-Q12-Q12`: `RollingMatching:Infeasible` — No feasible joint rolling matching exists (glpk error 10, status -1).

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
