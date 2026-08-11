# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `920a13269be66d5e235e194305bae9dfe1b82cdf`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `139.41 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+5.246%`
- Best tail-safe mean gain: `+0.000%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v106-protection-release-oracle-h8` | 1+2+4+5+3+6 | 2 | NaN | +0.400443 | 1 | +5.246% | +16.734% | -0.865% | +9.148% | +19.198% | +5.286% | 1 |

## Evidence boundary

V106 is a frozen retrospective protection-release headroom oracle. It starts from the exact V105 control-only protection schedule and releases F1 at t=77 and F6 at t=78, immediately before their opened V105 formation or peer outcomes reverse sign. Truth and opened V105 outcomes therefore define the release pages; V106 is not deployable and cannot support validation or generalization claims. Every topology adjacency and fusion-weight row stays equal to the matched static fixed-counter-clockwise route, and no gateway handoff occurs. The frozen H=8 static outcome is reused only after preset, seed, receiver mode, horizon, cache path and cache SHA-256 match. Cached inputs, measurements, delivery uniforms, filter RNG and the communication model are unchanged. V106 tests whether timely release has enough strict headroom to justify a later causal debt controller.
