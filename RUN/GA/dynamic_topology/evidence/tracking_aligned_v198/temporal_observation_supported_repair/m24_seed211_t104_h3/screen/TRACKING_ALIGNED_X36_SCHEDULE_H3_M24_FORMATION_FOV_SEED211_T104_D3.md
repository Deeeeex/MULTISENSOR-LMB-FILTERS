# Online positive-net payload H=3 return screen

- Contract: `tracking-aligned-x36-schedule-h3-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `m24-formation-fov / 211 / 104`
- Return times: `[104 105 106]`
- Intervention duration: `3` step(s)
- Generation commit: `fc74ad7bf11bb96b8153574328d8cebfd1fdbe78`
- Cache SHA-256: `93f03e13ddbb75f78a42d2b69a2d111490709964829e9ca811e8b0fd2b2a43c0`
- Bank construction: `38.46 s`
- Observable formation ranking: `[1 4 3 2]`
- Support-weighted rescue scores: `[0.107274 0.023205 0.0127549 0.00232155]`
- Addressable / total network risk: `3.442% / 3.499%`
- Initial positive-net action / formations: `1 / [1 3 4]`
- Initial addressable coverage / useful-loss ratio / net benefit: `100.000% / 0.903% / 0.03411`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+7.521%`
- Best tail-safe mean gain: `+7.521%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | RMSE gain | Worst gain | Min form. gain | Cons. gain | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v99-online-positive-net-f1-f3-f4` | 1+3+4 | 3 | NaN | +0.429701 | 1 | +7.521% | +10.620% | +26.255% | +0.000% | +17.429% | +3.562% | 1 |

## Evidence boundary

V198 keeps the V197 top-one repair token and two-page post-release cooldown. Before spending an available token, it evaluates set-entry support using the current and immediately preceding local-posterior pages, including currently reachable cross-edge senders. A token is spent only when the entered set remains unsupported over this causal window. The policy uses no truth, future measurement, future outcome or numeric formation identifier. Opened anchors remain development evidence only.
