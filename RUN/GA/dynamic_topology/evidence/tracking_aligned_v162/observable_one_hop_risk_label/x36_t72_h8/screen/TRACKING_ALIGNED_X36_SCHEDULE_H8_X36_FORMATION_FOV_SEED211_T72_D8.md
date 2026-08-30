# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `c166b6945960e2d88d343c15c0489e7f274d5fde`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `139.46 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+9.180%`
- Best tail-safe mean gain: `+9.180%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | RMSE gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v162-observable-one-hop-risk-label-k4` | 1+2+4+5+3+6 | 2 | NaN | +0.435100 | 1 | +9.180% | +7.918% | +19.320% | +5.915% | +11.509% | +16.402% | +2.458% | 1 |

## Evidence boundary

V162 is a paired recursive X36 seed-211 t=72 H=8 development screen. It retains the frozen V105 base route, fusion weights and protection schedule, and reuses the 36 opened receiver-time cells only as a trigger schedule. At each cell, current physical neighbors advertise present-time label keys and scalar posterior Bayes risks. The receiver chooses the minimum-risk one-hop source per label and requests at most four labels with positive receiver-minus-source risk. Responses carry complete Bernoulli Gaussian-mixture densities. The filter ledger charges synopsis, request, response-header and complete-label bytes on their directed physical links; all messages reuse the frozen per-link delivery realization and do not perturb the base communication RNG. Truth and future measurements do not enter any source or label decision. The opened trigger schedule remains privileged, so V162 is not an online method, validation or generalization evidence.
