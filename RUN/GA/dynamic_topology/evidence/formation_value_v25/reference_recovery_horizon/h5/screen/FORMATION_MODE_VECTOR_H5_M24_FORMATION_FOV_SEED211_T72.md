# Formation heterogeneous mode-vector H=5 return screen

- Contract: `formation-exhaustive-mode-vector-h5-opened-return-screen-v1`
- Preset / seed / time: `m24-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76]`
- Intervention: `explicit frozen H=5 sequence`
- Generation commit: `27b2c879c634a4e32705c53cf413caa7df9fb6d1`
- Cache SHA-256: `60dfbf2615181cde046af15f42bba37c415ea0034cb7ce53685b79042bfaf762`
- Bank construction: `74.02 s`
- Proxy positive / realized positive: `0 / 4`
- Proxy TP / FP / FN: `0 / 0 / 4`
- Proxy action agreement: `0.000`
- Best mean gain: `+11.289%`
- Best tail-safe mean gain: `+11.289%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `mode-vector-1-1-1-1 -> mode-vector-1-1-1-1 -> mode-vector-1-1-1-1 -> mode-vector-1-1-1-1 -> mode-vector-1-1-1-1` | 0 | NaN | 0.70 | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `mode-vector-1-1-3-1 -> mode-vector-1-1-1-4 -> mode-vector-1-4-4-2 -> mode-vector-1-1-1-1 -> mode-vector-1-1-1-1` | 3 | NaN | 0.65 | -0.005544 | 0 | +9.091% | +1.400% | +0.000% | -5.038% | -11.147% | -0.758% | 1 |
| `mode-vector-1-4-3-1 -> mode-vector-1-1-1-4 -> mode-vector-1-1-4-2 -> mode-vector-1-1-1-1 -> mode-vector-1-1-1-1` | 2+3 | NaN | 0.65 | -0.005544 | 0 | +9.735% | +1.400% | +0.000% | -3.282% | -9.238% | -0.079% | 1 |
| `mode-vector-1-4-3-1 -> mode-vector-1-1-1-4 -> mode-vector-1-1-1-2 -> mode-vector-1-1-1-1 -> mode-vector-1-1-1-1` | 2+3 | NaN | 0.65 | +0.000571 | 0 | +11.289% | +1.400% | +0.000% | -3.810% | -6.589% | -0.835% | 1 |
| `mode-vector-1-1-2-2 -> mode-vector-1-4-1-1 -> mode-vector-1-1-4-1 -> mode-vector-1-1-1-1 -> mode-vector-1-1-1-1` | 3+4 | NaN | 0.50 | -0.014913 | 0 | +4.740% | -1.619% | +0.000% | -2.800% | -4.824% | -0.128% | 1 |

## Evidence boundary

Every nonreference graph and fusion matrix in the explicit H=5 sequence is constructed once from the opened current posterior, link probabilities, payload estimates, and topology history. No future state rebuilds a frozen action; reference indices recompute only the registered fixed reference. Truth and future measurements score offline targets after execution. The opened state and sequence selection cannot support M24 or X36 validation claims.
