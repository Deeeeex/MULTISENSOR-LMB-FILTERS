# M24 CVaR graph policy failure decomposition

- Generated: 2026-07-30 02:48:22
- Commit: `e70eb047e85b191886df6d632004c78f05d61beb`
- Dataset-set SHA-256: `2eb144803806a2657e2fa74fa2a6b3a71008097ce5fa9006b64b60c8e286d511`
- Training seeds: `[11 17 19 23 27 29]`
- Development seeds opened: `0`
- Evidence boundary: This is an architecture-diagnosis analysis on the already-opened training seeds only. Oracle substitutions localize error and are not deployable policies. Formation candidates are exploratory training-LOSO diagnostics and cannot select a final model or open development labels.

## Structured action error decomposition

| Node head | Edge heads | Mean task adv. | Median oracle gap | P90 gap | Nonnegative | Tail safe | Fallback |
|:--|:--|--:|--:|--:|--:|--:|--:|
| learned | learned | -0.0574 | 0.2412 | 0.4080 | 0.278 | 0.806 | 0 |
| oracle | learned | -0.0651 | 0.2633 | 0.3513 | 0.194 | 1.000 | 0 |
| learned | oracle | 0.1849 | 0.0000 | 0.0926 | 0.972 | 0.944 | 0 |
| oracle | oracle | 0.2051 | 0.0000 | 0.0000 | 0.972 | 1.000 | 0 |

## Formation-risk LOSO diagnostics

| Features | Lambda | Top-1 | Min seed | Top-1 tail | Top-2 tail | Risk rho | Med regret | P90 |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|
| mean | 0.01 | 0.389 | 0.000 | 0.306 | 0.528 | 0.033 | 0.361 | 1.000 |
| mean | 0.1 | 0.361 | 0.000 | 0.315 | 0.556 | 0.111 | 0.361 | 1.000 |
| mean | 1 | 0.389 | 0.000 | 0.324 | 0.537 | 0.139 | 0.329 | 1.000 |
| mean | 10 | 0.444 | 0.167 | 0.315 | 0.556 | 0.111 | 0.088 | 1.000 |
| mean | 100 | 0.472 | 0.167 | 0.389 | 0.685 | 0.250 | 0.007 | 1.000 |
| mean-max | 0.01 | 0.444 | 0.000 | 0.352 | 0.574 | 0.222 | 0.017 | 1.000 |
| mean-max | 0.1 | 0.417 | 0.000 | 0.333 | 0.528 | 0.200 | 0.166 | 1.000 |
| mean-max | 1 | 0.444 | 0.167 | 0.333 | 0.528 | 0.244 | 0.162 | 1.000 |
| mean-max | 10 | 0.472 | 0.167 | 0.352 | 0.556 | 0.272 | 0.003 | 1.000 |
| mean-max | 100 | 0.500 | 0.167 | 0.389 | 0.657 | 0.333 | 0.004 | 1.000 |
| mean-max-min-std | 0.01 | 0.389 | 0.000 | 0.324 | 0.519 | 0.139 | 0.013 | 0.997 |
| mean-max-min-std | 0.1 | 0.389 | 0.000 | 0.324 | 0.519 | 0.139 | 0.013 | 0.997 |
| mean-max-min-std | 1 | 0.389 | 0.000 | 0.324 | 0.491 | 0.172 | 0.013 | 0.997 |
| mean-max-min-std | 10 | 0.472 | 0.000 | 0.380 | 0.565 | 0.267 | 0.001 | 1.000 |
| mean-max-min-std | 100 | 0.528 | 0.167 | 0.398 | 0.657 | 0.333 | 0.000 | 1.000 |

Best training-only diagnostic: `mean-max-min-std`, lambda `100`, top-1 accuracy `0.528`, top-1 tail coverage `0.398`.
