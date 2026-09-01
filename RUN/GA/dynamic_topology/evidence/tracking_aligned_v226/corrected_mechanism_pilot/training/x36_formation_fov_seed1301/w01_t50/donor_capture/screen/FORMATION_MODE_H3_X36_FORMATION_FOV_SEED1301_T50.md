# Formation-local H=3 opened return screen

- Contract: `direct-graph-payload-repair-h3-opened-return-screen-v214-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 1301 / 50`
- Return times: `[50 51 52]`
- Intervention duration: `1` step(s)
- Generation commit: `5fe8dd8478fc1f6e4b8ad7a57b8e3ad9d5102709`
- Cache SHA-256: `ad0b00ac1045f9b5397ffbd3bfe1112516693288bb1a79594eecf49c61b994e6`
- Bank construction: `12.00 s`
- Proxy positive / realized positive: `6 / 4`
- Proxy TP / FP / FN: `4 / 2 / 0`
- Proxy action agreement: `0.667`
- Best mean gain: `+0.086%`
- Best tail-safe mean gain: `+0.086%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | RMSE gain | Worst gain | Min form. gain | Cons. gain | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v214-withhold-full-posterior-f1-one-page` | 1 | 2 | NaN | -0.320701 | 1 | -0.001% | -0.352% | +0.000% | -0.007% | +0.010% | +0.277% | 1 |
| `v214-withhold-full-posterior-f2-one-page` | 2 | 3 | NaN | -0.356512 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.800% | 1 |
| `v214-withhold-full-posterior-f3-one-page` | 3 | 4 | NaN | -0.309260 | 1 | +0.061% | -0.242% | +0.000% | +0.000% | -0.380% | +0.529% | 1 |
| `v214-withhold-full-posterior-f4-one-page` | 4 | 5 | NaN | -0.334859 | 1 | +0.086% | -0.591% | +0.000% | +0.000% | -0.075% | -1.092% | 1 |
| `v214-withhold-full-posterior-f5-one-page` | 5 | 6 | NaN | -0.712539 | 1 | +0.086% | -0.452% | +0.000% | +0.000% | +0.197% | +2.424% | 1 |
| `v214-withhold-full-posterior-f6-one-page` | 6 | 7 | NaN | -0.648468 | 1 | +0.022% | -2.119% | +0.000% | +0.000% | +0.199% | +0.584% | 1 |

## Evidence boundary

V214 is frozen after observing that the V99 online arm required 1394.69 seconds versus 251.51 seconds for the paired X36 H=8 reference arm. It therefore removes V99 counterfactual enumeration from the runtime base and retains V99 only as an offline teacher. The new controller uses current observable graph features and exact byte accounting. New split trajectories remain development, calibration or held-out evidence according to the inherited V213 complete-trajectory split; no validation claim is yet allowed. The H=3 action bank withholds one receiver formation for the current page only, retains the complete Gaussian mixture for every posterior that is sent, and restores the full-payload reference on the following pages. Truth scores the paired return only and never enters the action.
