# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `2f7cf81acf6eba1658b1def11b05a7e21f96421b`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `138.70 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+5.237%`
- Best tail-safe mean gain: `+0.000%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v108-signed-complete-label-exception-h8` | 1+2+4+5+3+6 | 2 | NaN | +0.435100 | 1 | +5.237% | +16.734% | -0.930% | +9.643% | +17.658% | +5.870% | 1 |

## Evidence boundary

V108 is a frozen retrospective action-space headroom oracle. It keeps the exact V105 static route and control-only formation schedule but admits at most three complete Bernoulli Gaussian-mixture labels on an actually delivered F1 or F6 gateway edge. The label schedule was selected from opened V105 local posterior snapshots and target truth using positive one-round capped expected-risk reduction under the repository's componentwise powered-GM KLA approximation. It therefore uses truth and future opened states, is not deployable, and cannot support validation or generalization claims. Its sole purpose is to test whether sparse complete-label exceptions have enough headroom to repair V105's F1/F6 losses before a truth-free estimator or GNN is designed.
