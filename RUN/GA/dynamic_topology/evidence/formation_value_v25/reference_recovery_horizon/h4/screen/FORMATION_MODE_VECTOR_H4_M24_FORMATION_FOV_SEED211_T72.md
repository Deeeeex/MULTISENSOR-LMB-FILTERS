# Formation heterogeneous mode-vector H=4 return screen

- Contract: `formation-exhaustive-mode-vector-h4-opened-return-screen-v1`
- Preset / seed / time: `m24-formation-fov / 211 / 72`
- Return times: `[72 73 74 75]`
- Intervention: `explicit frozen H=4 sequence`
- Generation commit: `27b2c879c634a4e32705c53cf413caa7df9fb6d1`
- Cache SHA-256: `60dfbf2615181cde046af15f42bba37c415ea0034cb7ce53685b79042bfaf762`
- Bank construction: `73.81 s`
- Proxy positive / realized positive: `0 / 4`
- Proxy TP / FP / FN: `0 / 0 / 4`
- Proxy action agreement: `0.000`
- Best mean gain: `+9.293%`
- Best tail-safe mean gain: `+9.293%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `mode-vector-1-1-1-1 -> mode-vector-1-1-1-1 -> mode-vector-1-1-1-1 -> mode-vector-1-1-1-1` | 0 | NaN | 0.70 | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `mode-vector-1-1-3-1 -> mode-vector-1-1-1-4 -> mode-vector-1-4-4-2 -> mode-vector-1-1-1-1` | 3 | NaN | 0.65 | -0.005544 | 0 | +6.728% | +0.055% | +0.000% | -3.603% | -10.094% | -0.422% | 1 |
| `mode-vector-1-4-3-1 -> mode-vector-1-1-1-4 -> mode-vector-1-1-4-2 -> mode-vector-1-1-1-1` | 2+3 | NaN | 0.65 | -0.005544 | 0 | +7.128% | +0.082% | +0.000% | -1.883% | -2.444% | +0.224% | 1 |
| `mode-vector-1-4-3-1 -> mode-vector-1-1-1-4 -> mode-vector-1-1-1-2 -> mode-vector-1-1-1-1` | 2+3 | NaN | 0.65 | +0.000571 | 0 | +9.293% | -1.635% | +0.000% | -3.158% | -1.519% | -0.339% | 1 |
| `mode-vector-1-1-2-2 -> mode-vector-1-4-1-1 -> mode-vector-1-1-4-1 -> mode-vector-1-1-1-1` | 3+4 | NaN | 0.50 | -0.014913 | 0 | +4.258% | -0.005% | +0.000% | -2.325% | -4.357% | +0.040% | 1 |

## Evidence boundary

Every nonreference graph and fusion matrix in the explicit H=4 sequence is constructed once from the opened current posterior, link probabilities, payload estimates, and topology history. No future state rebuilds a frozen action; reference indices recompute only the registered fixed reference. Truth and future measurements score offline targets after execution. The opened state and sequence selection cannot support M24 or X36 validation claims.
