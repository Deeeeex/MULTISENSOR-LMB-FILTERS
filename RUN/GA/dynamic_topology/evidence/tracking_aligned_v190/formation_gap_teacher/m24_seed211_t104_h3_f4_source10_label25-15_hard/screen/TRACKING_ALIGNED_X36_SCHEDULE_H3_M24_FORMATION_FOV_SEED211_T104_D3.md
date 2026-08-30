# Online positive-net payload H=3 return screen

- Contract: `tracking-aligned-x36-schedule-h3-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `m24-formation-fov / 211 / 104`
- Return times: `[104 105 106]`
- Intervention duration: `3` step(s)
- Generation commit: `59380882c6f10770f49fd19728c30b91bd7c5097`
- Cache SHA-256: `93f03e13ddbb75f78a42d2b69a2d111490709964829e9ca811e8b0fd2b2a43c0`
- Bank construction: `56.13 s`
- Observable formation ranking: `[1 4 3 2]`
- Support-weighted rescue scores: `[0.107274 0.023205 0.0127549 0.00232155]`
- Addressable / total network risk: `3.442% / 3.499%`
- Initial positive-net action / formations: `1 / [1 3 4]`
- Initial addressable coverage / useful-loss ratio / net benefit: `100.000% / 0.903% / 0.03411`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+7.993%`
- Best tail-safe mean gain: `+7.993%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | RMSE gain | Worst gain | Min form. gain | Cons. gain | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v99-online-positive-net-f1-f3-f4` | 1+3+4 | 3 | NaN | +0.429701 | 1 | +7.993% | +2.002% | +28.584% | +0.000% | +19.410% | +4.040% | 1 |

## Evidence boundary

V99 keeps the matched static carrier graph and fusion weights, but recomputes the threshold-free safe positive-net receiver-formation set after every local update. Each decision uses the current pre-fusion posterior, current geometry and past selected graphs only; no truth, future measurements or future outcomes enter the selector. The static, fixed-V97 and online-V99 arms share the same cached posterior, measurements, link uniforms, filter RNG, horizon and communication constraints. Opened anchors remain development evidence only.
