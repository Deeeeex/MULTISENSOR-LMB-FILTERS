# Receiver-domain transport H=3 paired return screen

- Contract: `receiver-domain-transport-h3-opened-return-screen-v1`
- Preset / seed / time: `m24-formation-fov-merge-split / 1401 / 80`
- Return times: `[80 81 82]`
- Intervention: `explicit frozen H=3 sequence`
- Generation commit: `a7987f4b78746d31a5e3be961dbe44fde31adc72`
- Cache SHA-256: `c77a84f895bbff088f71a1c0cbe55daade927fccf1382ce51bdf8e28dfa1e20c`
- Bank construction: `54.79 s`
- Proxy positive / realized positive: `4 / 1`
- Proxy TP / FP / FN: `1 / 3 / 0`
- Proxy action agreement: `0.250`
- Best mean gain: `+0.668%`
- Best tail-safe mean gain: `+0.668%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Cons. gain | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|:--:|
| `reference -> reference -> reference` | 0 | 1 | 0.00 | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `receiver-domain-route-trust-05 -> reference -> receiver-domain-route-trust-05` | 3 | 2 | 0.05 | +2.039708 | 1 | +0.668% | +0.000% | +0.000% | -0.625% | -0.074% | 1 |
| `receiver-domain-route-trust-10 -> reference -> receiver-domain-route-trust-10` | 3 | 3 | 0.10 | +4.079416 | 1 | -0.027% | +0.000% | -0.151% | -0.707% | -0.074% | 1 |
| `receiver-domain-route-trust-15 -> reference -> receiver-domain-route-trust-15` | 3 | 4 | 0.15 | +6.119124 | 1 | -0.582% | +0.000% | -3.175% | -0.456% | +0.047% | 1 |
| `receiver-domain-route-trust-20 -> reference -> receiver-domain-route-trust-20` | 3 | 5 | 0.20 | +8.158832 | 1 | -0.748% | +0.000% | -4.078% | -0.392% | +0.059% | 1 |

## Evidence boundary

V72 opens exactly two already-frozen V71 source states: M24 t=80 and X36 t=52, seed 1401. Reference and candidate share the same predecision posterior, measurements, delivery draws, filter RNG, and current-physical-tree reference. The candidate executes one V71 fixed-message transport route at the anchor; both arms recompute the reference for the following two steps. Truth and future measurements score the paired H=3 outcome only after route construction. This is opened development evidence, not validation.
