# Formation coordinated-subset H=3 opened return screen

- Contract: `formation-local-pair-coordinated-subset-h3-opened-return-screen-v1`
- Preset / seed / time: `m24-formation-fov / 211 / 72`
- Return times: `[72 73 74]`
- Intervention: `explicit frozen H=3 sequence`
- Generation commit: `402b9a98fbee3cf785a53a54c8df650647fe8383`
- Cache SHA-256: `60dfbf2615181cde046af15f42bba37c415ea0034cb7ce53685b79042bfaf762`
- Bank construction: `73.91 s`
- Proxy positive / realized positive: `0 / 6`
- Proxy TP / FP / FN: `0 / 0 / 6`
- Proxy action agreement: `0.000`
- Best mean gain: `+8.647%`
- Best tail-safe mean gain: `+7.911%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Cons. gain | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|:--:|
| `reference -> reference -> reference` | 0 | 1 | 0.70 | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-4-dynamic-trust-0.70 -> reference` | 3 | 3 | 0.50 | +0.002253 | 0 | +7.911% | -0.001% | +0.000% | -6.069% | -0.844% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-4-dynamic-trust-0.70 -> formations-1-2-3-dynamic-trust-0.30` | 3 | 3 | 0.50 | -0.021517 | 0 | +5.726% | +0.031% | -0.041% | -5.125% | +0.981% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-4-dynamic-trust-0.70 -> formations-1-2-4-dynamic-trust-0.30` | 3 | 3 | 0.50 | -0.001511 | 0 | +8.647% | +0.031% | -0.041% | -3.778% | +1.059% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-4-dynamic-trust-0.70 -> formations-1-3-4-dynamic-trust-0.30` | 3 | 3 | 0.50 | +0.006799 | 0 | +6.407% | -0.001% | -0.041% | -2.837% | +1.633% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-4-dynamic-trust-0.70 -> formations-2-3-4-dynamic-trust-0.30` | 3 | 3 | 0.50 | -0.007397 | 0 | +6.448% | +0.031% | +0.000% | -2.395% | +1.027% | 1 |
| `formation-3-dynamic-trust-0.50 -> formation-4-dynamic-trust-0.70 -> formations-1-2-3-4-dynamic-trust-0.30` | 3 | 3 | 0.50 | -0.008627 | 0 | +6.439% | +0.031% | -0.041% | -2.637% | +1.848% | 1 |

## Evidence boundary

Every nonreference graph and fusion matrix in the explicit H=3 sequence is constructed once from the opened current posterior, link probabilities, payload estimates, and topology history. No future state rebuilds a frozen action; reference indices recompute only the registered fixed reference. Truth and future measurements score offline targets after execution. The opened state and sequence selection cannot support M24 or X36 validation claims.
