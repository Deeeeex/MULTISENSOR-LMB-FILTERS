# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `48ee36a8168fae4c83840bbf5bb86c72a214d62d`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `139.15 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+5.204%`
- Best tail-safe mean gain: `+0.000%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v111-alternating-shield-broadcast-h8` | 1+2+4+5+3+6 | 2 | NaN | +0.435100 | 1 | +5.204% | +16.734% | -0.816% | +10.685% | +20.684% | +5.483% | 1 |

## Evidence boundary

V111 is a frozen H=8 propagation-control headroom probe. It extends the V102 one-step-delayed alternating shield/broadcast cadence to the complete V105 H=8 protection schedule and uses explicit abstention for protected cross inputs. Broadcast and reference recovery pages alternate, every route page remains physically reachable, and the matched static full-payload outcome is reused. The schedule is constructed without V110 outcomes, but it remains frozen development evidence rather than an online policy or a validation/generalization claim.
