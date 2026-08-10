# Receiver-domain transport H=3 paired return screen

- Contract: `receiver-domain-transport-h3-opened-return-screen-v1`
- Preset / seed / time: `x36-formation-fov-merge-split / 1401 / 52`
- Return times: `[52 53 54]`
- Intervention duration: `1` step(s)
- Generation commit: `34e3850383cb056f63bb46593248fce11d00c0c3`
- Cache SHA-256: `a5d377e169ad924380f8eef644bcd3a196c7e95224680b1d85278c2d3e1872ca`
- Bank construction: `209.44 s`
- Proxy positive / realized positive: `1 / 0`
- Proxy TP / FP / FN: `0 / 1 / 0`
- Proxy action agreement: `0.000`
- Best mean gain: `+0.000%`
- Best tail-safe mean gain: `+0.000%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Cons. gain | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|:--:|
| `reference` | 0 | 1 | 0.00 | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `receiver-domain-transport-f4-f5` | 4+5 | 2 | 0.05 | +4.296950 | 1 | -0.290% | +0.000% | -2.399% | +0.142% | -0.013% | 1 |

## Evidence boundary

V72 opens exactly two already-frozen V71 source states: M24 t=80 and X36 t=52, seed 1401. Reference and candidate share the same predecision posterior, measurements, delivery draws, filter RNG, and current-physical-tree reference. The candidate executes one V71 fixed-message transport route at the anchor; both arms recompute the reference for the following two steps. Truth and future measurements score the paired H=3 outcome only after route construction. This is opened development evidence, not validation.
