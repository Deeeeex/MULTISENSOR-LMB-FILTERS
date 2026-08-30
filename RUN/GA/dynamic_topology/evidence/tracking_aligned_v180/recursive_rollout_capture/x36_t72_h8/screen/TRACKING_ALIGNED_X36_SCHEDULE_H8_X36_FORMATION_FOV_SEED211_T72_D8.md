# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `8f01b9e1978ad3adfd2ea382f2ca30b2097eb201`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `141.26 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+11.262%`
- Best tail-safe mean gain: `+11.262%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | RMSE gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v179-analytic-f3-plus-rollout-aware-f5-repair` | 1+2+4+5+3+6 | 2 | NaN | +0.435100 | 1 | +11.262% | +10.673% | +19.320% | +6.112% | +9.823% | +13.868% | -0.027% | 1 |

## Evidence boundary

V179 is a paired recursive X36 seed-211 t=72 H=8 policy-iteration screen. It retains the V169 route, weights, protection schedule and truth-free analytic transfers outside F5. At the F5 t=78/79 cells, one compact classifier trained on V166 static states plus V176 t=78 states visited by recursive V169 selects at most one complete Bernoulli GM label. Hyperparameters and the probability threshold were frozen with V166 F3 calibration before the V176 F5 t=79 rollout cells were opened. Inference reads no truth, future measurements or numeric label IDs. The ledger charges a 24-byte packed synopsis per advertised active source-label, a request and the complete-label response. All data come from opened seed 211, so this is recursive method-development evidence, not validation or a generalization claim.
