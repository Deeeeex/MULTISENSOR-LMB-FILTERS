# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `3c4add8af40c0603206f2db4e34c14935ec76870`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `140.20 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+5.179%`
- Best tail-safe mean gain: `+0.000%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v107-protection-early-release-oracle-h8` | 1+2+4+5+3+6 | 2 | NaN | +0.388613 | 1 | +5.179% | +16.569% | -0.865% | +8.818% | +17.288% | +4.848% | 1 |

## Evidence boundary

V107 is a frozen retrospective one-page-early release oracle. It starts from V105 but releases F1 at t=76 and F6 at t=77, one page before the opened V105 formation or peer outcomes first reverse sign. Truth and opened V105 outcomes define these pages, so V107 is not deployable and cannot support validation or generalization claims. Every topology adjacency and fusion-weight row remains the matched static fixed-counter-clockwise route and no handoff occurs. The frozen H=8 static outcome is reused only after preset, seed, receiver mode, horizon, cache path and cache SHA-256 match. V107 distinguishes a late reactive release from a structural failure of binary formation-level full/control-only switching.
