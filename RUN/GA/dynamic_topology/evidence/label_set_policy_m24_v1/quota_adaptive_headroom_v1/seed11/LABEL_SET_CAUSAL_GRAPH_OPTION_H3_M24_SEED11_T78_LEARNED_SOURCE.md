# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `11 / 78`
- Audit mode: `quota-adaptive-headroom`
- Return times: `[78 79 80]`
- Source SHA-256: `82404ba0ae5b65b9d4ddad6c1b7d91191630d6f0e98eae4a55b42875fd1665c2`
- Source behavior: `label-set-message-passing-safe-fixed-e05-a70`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `Q3-Q3-Q3`
- Best gain versus CCC: `2.782%`

- Registered projection failures: `2`

| Option | Status | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | completed | 15.371985 | 52.446663 | 0.430556 | 21.153136 | 6032640 | +0.000% | 1 |
| `LLL` | completed | 17.394177 | 44.527197 | 0.555556 | 23.579707 | 6090936 | +0.966% | 1 |
| `Q3-Q3-Q3` | completed | 14.944265 | 38.769376 | 0.388889 | 20.055035 | 6112872 | +1.330% | 1 |
| `Q4-Q4-Q4` | completed | 16.284873 | 38.769376 | 0.458333 | 21.735776 | 6094032 | +1.018% | 1 |
| `Q6-Q6-Q6` | completed | 16.949476 | 45.061763 | 0.513889 | 23.116126 | 6067296 | +0.574% | 1 |
| `Q8-Q8-Q8` | completed | 19.234870 | 45.055740 | 0.638889 | 24.976877 | 6072672 | +0.664% | 1 |
| `Q10-Q10-Q10` | infeasible | Inf | Inf | Inf | Inf | NaN | +NaN% | 0 |
| `Q12-Q12-Q12` | infeasible | Inf | Inf | Inf | Inf | NaN | +NaN% | 0 |

## Hard failures

- `Q10-Q10-Q10`: `RollingMatching:Infeasible` — No feasible joint rolling matching exists (glpk error 10, status -1).
- `Q12-Q12-Q12`: `RollingMatching:Infeasible` — No feasible joint rolling matching exists (glpk error 10, status -1).

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
