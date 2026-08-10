# Receiver-domain transport H=3 paired return screen

- Contract: `receiver-domain-transport-h3-opened-return-screen-v1`
- Preset / seed / time: `m24-formation-fov-merge-split / 1401 / 80`
- Return times: `[80 81 82]`
- Intervention duration: `1` step(s)
- Generation commit: `34e3850383cb056f63bb46593248fce11d00c0c3`
- Cache SHA-256: `c77a84f895bbff088f71a1c0cbe55daade927fccf1382ce51bdf8e28dfa1e20c`
- Bank construction: `54.04 s`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+0.473%`
- Best tail-safe mean gain: `+0.473%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Cons. gain | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|:--:|
| `reference` | 0 | 1 | 0.00 | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `receiver-domain-transport-f3` | 3 | 2 | 0.05 | +2.039708 | 1 | +0.473% | +0.000% | +0.000% | -0.528% | -0.158% | 1 |

## Evidence boundary

V72 opens exactly two already-frozen V71 source states: M24 t=80 and X36 t=52, seed 1401. Reference and candidate share the same predecision posterior, measurements, delivery draws, filter RNG, and current-physical-tree reference. The candidate executes one V71 fixed-message transport route at the anchor; both arms recompute the reference for the following two steps. Truth and future measurements score the paired H=3 outcome only after route construction. This is opened development evidence, not validation.
