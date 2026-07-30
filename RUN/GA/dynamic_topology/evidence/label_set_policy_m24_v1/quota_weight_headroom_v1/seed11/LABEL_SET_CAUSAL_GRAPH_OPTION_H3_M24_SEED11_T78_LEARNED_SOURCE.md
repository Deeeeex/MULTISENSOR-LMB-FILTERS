# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `11 / 78`
- Audit mode: `quota-weight-headroom`
- Return times: `[78 79 80]`
- Source SHA-256: `82404ba0ae5b65b9d4ddad6c1b7d91191630d6f0e98eae4a55b42875fd1665c2`
- Source behavior: `label-set-message-passing-safe-fixed-e05-a70`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `CCC`
- Best gain versus CCC: `0.000%`

- Registered projection failures: `0`

| Option | Status | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | completed | 15.371985 | 52.446663 | 0.430556 | 21.153136 | 6032640 | +0.000% | 1 |
| `LLL` | completed | 17.394177 | 44.527197 | 0.555556 | 23.579707 | 6090936 | +0.966% | 1 |
| `Q3E10-Q3E10-Q3E10` | completed | 16.509132 | 41.829002 | 0.486111 | 21.971630 | 6116232 | +1.386% | 1 |
| `Q3E20-Q3E20-Q3E20` | completed | 17.925392 | 44.517777 | 0.541667 | 23.361846 | 6112800 | +1.329% | 1 |
| `Q3E25-Q3E25-Q3E25` | completed | 18.984263 | 46.844893 | 0.638889 | 25.181893 | 6097248 | +1.071% | 1 |
| `Q6E10-Q6E10-Q6E10` | completed | 19.357540 | 44.994389 | 0.638889 | 25.374048 | 6072360 | +0.658% | 1 |
| `Q6E20-Q6E20-Q6E20` | completed | 19.684935 | 50.283812 | 0.736111 | 27.271836 | 6053928 | +0.353% | 1 |
| `Q6E25-Q6E25-Q6E25` | completed | 19.973653 | 52.641646 | 0.750000 | 27.771403 | 6051864 | +0.319% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
