# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `17 / 79`
- Audit mode: `behavior-distribution-shift`
- Return times: `[79 80 81]`
- Source SHA-256: `bcd1bad02344e9b54d0e55e6e9b65c1c2204863eafe6069b8497a7e5bd84bb0c`
- Source behavior: `fixed-counter-clockwise`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `LLL`
- Best gain versus CCC: `5.449%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 19.160199 | 34.829506 | 0.541667 | 20.084058 | 5985120 | +0.000% | 1 |
| `LLL` | 18.116064 | 32.293448 | 0.486111 | 19.954283 | 5999424 | +0.239% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
