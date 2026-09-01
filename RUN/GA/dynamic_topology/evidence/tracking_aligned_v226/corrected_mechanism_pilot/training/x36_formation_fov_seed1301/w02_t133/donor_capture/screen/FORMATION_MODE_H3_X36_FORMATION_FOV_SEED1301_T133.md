# Formation-local H=3 opened return screen

- Contract: `direct-graph-payload-repair-h3-opened-return-screen-v214-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 1301 / 133`
- Return times: `[133 134 135]`
- Intervention duration: `1` step(s)
- Generation commit: `5fe8dd8478fc1f6e4b8ad7a57b8e3ad9d5102709`
- Cache SHA-256: `773059b873084445d62500fd699c30dbb706c9f13cb986ca430c9e885e689fcf`
- Bank construction: `12.35 s`
- Proxy positive / realized positive: `6 / 2`
- Proxy TP / FP / FN: `2 / 4 / 0`
- Proxy action agreement: `0.333`
- Best mean gain: `+0.372%`
- Best tail-safe mean gain: `+0.372%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | RMSE gain | Worst gain | Min form. gain | Cons. gain | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v214-withhold-full-posterior-f1-one-page` | 1 | 2 | NaN | -0.849046 | 1 | +0.184% | +0.465% | +0.000% | +0.000% | +0.547% | +0.404% | 1 |
| `v214-withhold-full-posterior-f2-one-page` | 2 | 3 | NaN | -0.389587 | 1 | -0.037% | -0.307% | +0.000% | -0.232% | +0.003% | +0.413% | 1 |
| `v214-withhold-full-posterior-f3-one-page` | 3 | 4 | NaN | -0.770327 | 1 | -0.001% | -0.019% | +0.000% | -0.005% | -0.002% | +1.301% | 1 |
| `v214-withhold-full-posterior-f4-one-page` | 4 | 5 | NaN | -0.765368 | 1 | -0.004% | -0.105% | +0.000% | -0.025% | -0.069% | +0.183% | 1 |
| `v214-withhold-full-posterior-f5-one-page` | 5 | 6 | NaN | -1.048180 | 1 | +0.372% | +0.022% | +2.877% | +0.000% | +0.400% | +0.025% | 1 |
| `v214-withhold-full-posterior-f6-one-page` | 6 | 7 | NaN | -0.814243 | 1 | -0.009% | -0.763% | +0.000% | -0.050% | +0.175% | +0.485% | 1 |

## Evidence boundary

V214 is frozen after observing that the V99 online arm required 1394.69 seconds versus 251.51 seconds for the paired X36 H=8 reference arm. It therefore removes V99 counterfactual enumeration from the runtime base and retains V99 only as an offline teacher. The new controller uses current observable graph features and exact byte accounting. New split trajectories remain development, calibration or held-out evidence according to the inherited V213 complete-trajectory split; no validation claim is yet allowed. The H=3 action bank withholds one receiver formation for the current page only, retains the complete Gaussian mixture for every posterior that is sent, and restores the full-payload reference on the following pages. Truth scores the paired return only and never enters the action.
