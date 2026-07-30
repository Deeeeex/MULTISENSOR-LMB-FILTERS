# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `27 / 78`
- Audit mode: `quota-adaptive-headroom`
- Return times: `[78 79 80]`
- Source SHA-256: `3c7ef8d12001346e5acdf254558e2aaa1df488c9e360b5b9214877238643b94b`
- Source behavior: `label-set-message-passing-safe-fixed-e05-a70`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `Q6-Q6-Q6`
- Best gain versus CCC: `3.461%`

- Registered projection failures: `1`

| Option | Status | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | completed | 17.483976 | 32.310833 | 0.458333 | 20.086105 | 6031560 | +0.000% | 1 |
| `LLL` | completed | 16.894561 | 30.932400 | 0.416667 | 20.109982 | 6044496 | +0.214% | 1 |
| `Q3-Q3-Q3` | completed | 16.887742 | 30.948513 | 0.416667 | 20.104590 | 6049728 | +0.301% | 1 |
| `Q4-Q4-Q4` | completed | 16.891217 | 30.951557 | 0.416667 | 20.109214 | 6045216 | +0.226% | 1 |
| `Q6-Q6-Q6` | completed | 16.878856 | 30.980698 | 0.416667 | 20.097238 | 6040776 | +0.153% | 1 |
| `Q8-Q8-Q8` | completed | 16.881534 | 30.933101 | 0.416667 | 20.102877 | 6055488 | +0.397% | 1 |
| `Q10-Q10-Q10` | completed | 17.539133 | 30.933101 | 0.458333 | 20.600556 | 6047880 | +0.271% | 1 |
| `Q12-Q12-Q12` | infeasible | Inf | Inf | Inf | Inf | NaN | +NaN% | 0 |

## Hard failures

- `Q12-Q12-Q12`: `RollingMatching:Infeasible` — No feasible joint rolling matching exists (glpk error 10, status -1).

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
