# Receiver-domain transport H=3 paired return screen

- Contract: `receiver-domain-transport-h3-opened-return-screen-v1`
- Preset / seed / time: `x36-formation-fov-merge-split / 1401 / 52`
- Return times: `[52 53 54]`
- Intervention: `explicit frozen H=3 sequence`
- Generation commit: `a7987f4b78746d31a5e3be961dbe44fde31adc72`
- Cache SHA-256: `a5d377e169ad924380f8eef644bcd3a196c7e95224680b1d85278c2d3e1872ca`
- Bank construction: `210.52 s`
- Proxy positive / realized positive: `4 / 2`
- Proxy TP / FP / FN: `2 / 2 / 0`
- Proxy action agreement: `0.500`
- Best mean gain: `+0.201%`
- Best tail-safe mean gain: `+0.201%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Cons. gain | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|:--:|
| `reference -> reference -> reference` | 0 | 1 | 0.00 | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `receiver-domain-route-trust-05 -> reference -> receiver-domain-route-trust-05` | 4 | 2 | 0.05 | +1.099191 | 1 | +0.201% | +0.000% | +0.000% | +0.295% | -0.006% | 1 |
| `receiver-domain-route-trust-10 -> reference -> receiver-domain-route-trust-10` | 4 | 3 | 0.10 | +2.198382 | 1 | +0.118% | +0.000% | +0.000% | +0.253% | +0.075% | 1 |
| `receiver-domain-route-trust-15 -> reference -> receiver-domain-route-trust-15` | 4 | 4 | 0.15 | +3.297574 | 1 | -0.034% | +0.000% | -0.219% | -0.012% | +0.075% | 1 |
| `receiver-domain-route-trust-20 -> reference -> receiver-domain-route-trust-20` | 4 | 5 | 0.20 | +4.396765 | 1 | -0.183% | +0.000% | -1.173% | -0.156% | +0.075% | 1 |

## Evidence boundary

V72 opens exactly two already-frozen V71 source states: M24 t=80 and X36 t=52, seed 1401. Reference and candidate share the same predecision posterior, measurements, delivery draws, filter RNG, and current-physical-tree reference. The candidate executes one V71 fixed-message transport route at the anchor; both arms recompute the reference for the following two steps. Truth and future measurements score the paired H=3 outcome only after route construction. This is opened development evidence, not validation.
