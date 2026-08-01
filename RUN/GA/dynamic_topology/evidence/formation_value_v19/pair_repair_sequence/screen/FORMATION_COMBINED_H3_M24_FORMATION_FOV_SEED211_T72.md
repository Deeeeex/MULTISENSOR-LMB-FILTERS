# Formation combined local-plus-pair H=3 return screen

- Contract: `formation-single-plus-pair-h3-opened-return-screen-v1`
- Preset / seed / time: `m24-formation-fov / 211 / 72`
- Return times: `[72 73 74]`
- Intervention: `explicit frozen H=3 sequence`
- Generation commit: `ec9541cf580c616456c44d1dea5d28d0a4d8b631`
- Cache SHA-256: `60dfbf2615181cde046af15f42bba37c415ea0034cb7ce53685b79042bfaf762`
- Bank construction: `73.67 s`
- Proxy positive / realized positive: `0 / 7`
- Proxy TP / FP / FN: `0 / 0 / 7`
- Proxy action agreement: `0.000`
- Best mean gain: `+8.656%`
- Best tail-safe mean gain: `+8.656%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Cons. gain | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|:--:|
| `reference -> reference -> reference` | 0 | 1 | 0.70 | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-4-dynamic-trust-0.70 -> reference` | 3 | 3 | 0.50 | +0.002253 | 0 | +7.911% | -0.001% | +0.000% | -6.069% | -0.844% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-4-dynamic-trust-0.70 -> formations-1-2-dynamic-trust-0.30` | 3 | 3 | 0.50 | -0.014402 | 0 | +7.934% | +0.031% | -0.041% | -6.138% | +0.192% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-4-dynamic-trust-0.70 -> formations-1-3-dynamic-trust-0.30` | 3 | 3 | 0.50 | -0.006092 | 0 | +5.694% | -0.001% | -0.041% | -5.262% | +0.766% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-4-dynamic-trust-0.70 -> formations-1-4-dynamic-trust-0.30` | 3 | 3 | 0.50 | +0.013914 | 0 | +8.615% | -0.001% | -0.041% | -3.968% | +0.844% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-4-dynamic-trust-0.70 -> formations-2-3-dynamic-trust-0.30` | 3 | 3 | 0.50 | -0.020287 | 0 | +5.735% | +0.031% | +0.000% | -4.914% | +0.160% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-4-dynamic-trust-0.70 -> formations-2-4-dynamic-trust-0.30` | 3 | 3 | 0.50 | -0.000281 | 0 | +8.656% | +0.031% | +0.000% | -3.554% | +0.238% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-4-dynamic-trust-0.70 -> formations-3-4-dynamic-trust-0.30` | 3 | 3 | 0.50 | +0.008029 | 0 | +6.416% | -0.001% | +0.000% | -2.589% | +0.812% | 1 |

## Evidence boundary

Every nonreference graph and fusion matrix in the explicit H=3 sequence is constructed once from the opened current posterior, link probabilities, payload estimates, and topology history. No future state rebuilds a frozen action; reference indices recompute only the registered fixed reference. Truth and future measurements score offline targets after execution. The opened state and sequence selection cannot support M24 or X36 validation claims.
