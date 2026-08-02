# V27 reference-retention signal audit

- Contract: `formation-v27-reference-retention-signal-audit-v1`
- Source generation commit: `9d9fc7e003a868f688718061aac5c01eebb0b78d`
- Source cache SHA-256: `60dfbf2615181cde046af15f42bba37c415ea0034cb7ce53685b79042bfaf762`
- New tracking state opened: `0`
- Signal truth/future use: `0 / 0`

| Action | Mix gap | One-step disagreement | Retention risk | Min formation d-card | Min label retention | Threshold drops | Bayes-risk advantage | Tracking gain | Actual card. error |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| `reference-cycle-cross-w0p05` | 0.008298 | 2.001740 | 0.000000 | +0.000000 | 1.000000 | 0 | +0.000000 | +0.000% | 2.708333 |
| `cycle-cross-w0p10` | 0.016525 | 1.978307 | 0.029702 | -0.185293 | 0.198364 | 2 | -0.000686 | -1.318% | 2.819444 |
| `cycle-cross-w0p15` | 0.024680 | 1.955266 | 0.076662 | -0.478110 | 0.082356 | 5 | -0.002025 | -3.170% | 2.972222 |
| `dual-gateway-cycle-cross-w0p05` | 0.016525 | 1.890362 | 0.086328 | -0.827111 | 0.029412 | 10 | -0.002151 | -6.848% | 3.166667 |
| `dual-gateway-cycle-cross-w0p10` | 0.032759 | 1.831750 | 0.138477 | -1.231834 | 0.029415 | 17 | -0.002584 | -10.999% | 3.486111 |
| `bidirectional-cycle-cross-w0p05` | 0.016667 | 1.910296 | 0.070851 | -0.697551 | 0.060650 | 7 | -0.000171 | -5.972% | 3.055556 |
| `bidirectional-cycle-cross-w0p10` | 0.033333 | 1.848049 | 0.107319 | -1.044418 | 0.060651 | 14 | -0.002432 | -7.325% | 3.194444 |
| `cycle-antipodal-cross-w0p05` | 0.016667 | 1.855119 | 0.069181 | -0.672488 | 0.034628 | 15 | -0.006460 | -8.358% | 3.250000 |
| `cycle-antipodal-cross-w0p10` | 0.033333 | 1.772191 | 0.116049 | -1.032966 | 0.034632 | 23 | -0.003808 | -11.465% | 3.527778 |
| `bidirectional-antipodal-cross-w0p05` | 0.033333 | 1.766196 | 0.150773 | -1.467959 | 0.035344 | 22 | -0.005666 | -15.442% | 3.666667 |
| `bidirectional-antipodal-cross-w0p10` | 0.066667 | 1.655515 | 0.206061 | -1.986302 | 0.035348 | 35 | -0.007985 | -19.433% | 4.013889 |

## Within-state diagnostic alignment

- Retention risk vs. tracking loss: `r = 0.945721`
- Minimum formation expected-cardinality change vs. tracking gain: `r = 0.965253`
- Existing posterior Bayes-risk advantage vs. tracking gain: `r = 0.817044`
- One-step disagreement improvement vs. tracking gain: `r = -0.979107`
- Retention risk vs. actual cardinality-error increase: `r = 0.956578`

## Interpretation boundary

All candidate signals use only the already-opened current posterior, current physical graph, and current delivery probabilities. Saved tracking outcomes are read only after scoring to diagnose alignment. The action-level correlations share one state and are not generalization evidence or independent samples.

The new signal is an asymmetric safety diagnostic: it penalizes loss of reference-supported existence mass but does not claim the reference is ground truth. It is eligible for v28 only if this audit shows materially better mechanism alignment than the existing internal Bayes-risk and disagreement objectives. Thresholds and the new pairwise action space must be frozen before another tracking continuation is executed.

Frozen v27 strong candidates remain `0/10`.
