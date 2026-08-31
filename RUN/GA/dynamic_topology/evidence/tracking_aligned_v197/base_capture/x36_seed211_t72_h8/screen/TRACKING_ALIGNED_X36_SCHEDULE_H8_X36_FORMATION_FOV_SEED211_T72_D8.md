# Online positive-net payload H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `b14df427bb8ad18cb2101288ceb2d48d715c951b`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `146.37 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Addressable / total network risk: `0.752% / 0.907%`
- Initial positive-net action / formations: `1 / [1 2 4 5]`
- Initial addressable coverage / useful-loss ratio / net benefit: `100.000% / 14.505% / 0.00643`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+5.457%`
- Best tail-safe mean gain: `+0.000%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | RMSE gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +NaN% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v99-online-positive-net-f1-f2-f4-f5` | 1+2+4+5 | 3 | NaN | +0.370940 | 1 | +5.457% | +NaN% | +16.734% | -0.931% | +8.803% | +14.274% | +5.423% | 1 |

## Evidence boundary

V99 keeps the matched static carrier graph and fusion weights, but recomputes the threshold-free safe positive-net receiver-formation set after every local update. Each decision uses the current pre-fusion posterior, current geometry and past selected graphs only; no truth, future measurements or future outcomes enter the selector. The static, fixed-V97 and online-V99 arms share the same cached posterior, measurements, link uniforms, filter RNG, horizon and communication constraints. Opened anchors remain development evidence only.
