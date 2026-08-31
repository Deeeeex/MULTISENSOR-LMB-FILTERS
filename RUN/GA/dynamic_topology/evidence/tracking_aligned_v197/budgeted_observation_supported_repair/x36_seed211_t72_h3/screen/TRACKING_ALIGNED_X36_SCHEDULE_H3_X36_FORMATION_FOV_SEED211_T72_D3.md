# Online positive-net payload H=3 return screen

- Contract: `tracking-aligned-x36-schedule-h3-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74]`
- Intervention duration: `3` step(s)
- Generation commit: `7e028039c217f21a4adfdd1a98b8b26094fd9490`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `144.98 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Addressable / total network risk: `0.752% / 0.907%`
- Initial positive-net action / formations: `1 / [1 2 4 5]`
- Initial addressable coverage / useful-loss ratio / net benefit: `100.000% / 14.505% / 0.00643`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+2.181%`
- Best tail-safe mean gain: `+2.181%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | RMSE gain | Worst gain | Min form. gain | Cons. gain | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v99-online-positive-net-f1-f2-f4-f5` | 1+2+4+5 | 3 | NaN | +0.139103 | 1 | +2.181% | +0.589% | +0.000% | +0.000% | +2.648% | +5.397% | 1 |

## Evidence boundary

V197 computes the current V99 omission proposal and the V194 observation-unsupported set-entry projection. If no full-posterior repair occurred during the preceding two completed pages, it restores at most one proposed formation: the formation with the largest current maximum receiver set-entry risk. The ordinary message builder and byte ledger execute and charge the projected payloads. The policy sees only current observable posteriors and its own past release decisions; it uses no truth, future measurement or future outcome. Opened anchors remain development evidence only.
