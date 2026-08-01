# Formation-local H=3 opened return screen

- Contract: `formation-local-h3-opened-return-screen-v1`
- Preset / seed / time: `m24-formation-fov / 211 / 72`
- Return times: `[72 73 74]`
- Intervention: `explicit frozen H=3 sequence`
- Generation commit: `b38a4137ed48607d7fc72b2ae473fc3b79c098ab`
- Cache SHA-256: `60dfbf2615181cde046af15f42bba37c415ea0034cb7ce53685b79042bfaf762`
- Bank construction: `74.18 s`
- Proxy positive / realized positive: `0 / 10`
- Proxy TP / FP / FN: `0 / 0 / 10`
- Proxy action agreement: `0.000`
- Best mean gain: `+7.911%`
- Best tail-safe mean gain: `+7.911%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Cons. gain | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|:--:|
| `reference -> reference -> reference` | 0 | 1 | 0.70 | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `formation-3-dynamic-trust-0.50 -> reference -> reference` | 3 | 3 | 0.50 | -0.007512 | 0 | +5.988% | -0.001% | +0.000% | -11.486% | +0.167% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-1-dynamic-trust-0.30 -> reference` | 3 | 3 | 0.50 | -0.008741 | 0 | +5.697% | -0.001% | -1.320% | -12.761% | +0.803% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-1-dynamic-trust-0.50 -> reference` | 3 | 3 | 0.50 | -0.006648 | 0 | +5.704% | -0.001% | -1.288% | -12.761% | +0.790% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-1-dynamic-trust-0.70 -> reference` | 3 | 3 | 0.50 | -0.006705 | 0 | +5.704% | -0.001% | -1.287% | -12.847% | +0.815% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-2-dynamic-trust-0.30 -> reference` | 3 | 3 | 0.50 | -0.022937 | 0 | +6.015% | -0.013% | +0.000% | -11.546% | +0.267% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-2-dynamic-trust-0.50 -> reference` | 3 | 3 | 0.50 | -0.025015 | 0 | +5.750% | -0.003% | -0.934% | -12.320% | +0.267% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-2-dynamic-trust-0.70 -> reference` | 3 | 3 | 0.50 | -0.022084 | 0 | +6.004% | +0.000% | +0.000% | -11.863% | +0.255% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-4-dynamic-trust-0.30 -> reference` | 3 | 3 | 0.50 | +0.005379 | 0 | +7.062% | -2.208% | +0.000% | -8.297% | -0.896% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-4-dynamic-trust-0.50 -> reference` | 3 | 3 | 0.50 | +0.003987 | 0 | +7.231% | -0.001% | +0.000% | -7.754% | -0.878% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-4-dynamic-trust-0.70 -> reference` | 3 | 3 | 0.50 | +0.002253 | 0 | +7.911% | -0.001% | +0.000% | -6.069% | -0.844% | 1 |

## Evidence boundary

Every nonreference graph and fusion matrix in the explicit H=3 sequence is constructed once from the opened current posterior, link probabilities, payload estimates, and topology history. No future state rebuilds a frozen action; reference indices recompute only the registered fixed reference. Truth and future measurements score offline targets after execution. The opened state and sequence selection cannot support M24 or X36 validation claims.
