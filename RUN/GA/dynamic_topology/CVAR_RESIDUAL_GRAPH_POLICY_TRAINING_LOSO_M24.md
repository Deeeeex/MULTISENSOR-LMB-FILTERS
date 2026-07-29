# M24 CVaR relational graph policy: training LOSO

- Generated: 2026-07-30 02:43:22
- Training commit: `94bab9a7155917989943cd285fc996285fbb09eb`
- Dataset-set SHA-256: `2eb144803806a2657e2fa74fa2a6b3a71008097ce5fa9006b64b60c8e286d511`
- Feature contract SHA-256: `8f54c68fd00b1f86fc38c2d48b8f2c7848f74f9565ec1457a0e39bb1a071c721`
- Training seeds: `[11 17 19 23 27 29]`
- Evaluation: `whole-seed-fit4-calibrate1-test1-outer-loso`
- Selected family: `ridge`
- Selection reason: `deterministic-ridge-sentinel-retained`
- Final fit seeds: `[11 17 19 23 27]`
- Final calibration seed: `29`
- Safety margin: `42.5311468`
- Training LOSO passed: `0`
- Development labels opened: `0`
- Development evaluation authorized: `0`
- H=3 return generation authorized: `0`

| Family | Hard | Tail recall | Min seed | Tail Jaccard | Task rho | Min seed | Regret med | Regret P90 | False-safe | Safe recall | Selected safe | Fallback |
|:--|:--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ridge | 0 | 0.157 | 0.111 | 0.100 | 0.339 | 0.180 | 0.351 | 0.530 | 0.748 | 0.040 | 0.806 | 0 |
| mlp | 0 | 0.167 | 0.000 | 0.106 | 0.401 | 0.224 | 0.372 | 0.575 | 0.301 | 0.018 | 0.778 | 0 |

## Selected-family seed macro

| Seed | States | Tail recall | Jaccard | Task rho | Regret med | Regret P90 | Mean task gain | Nonnegative states | Fallback |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| 11 | 6 | 0.111 | 0.067 | 0.559 | 0.285 | 0.511 | -0.165057 | 0 | 0 |
| 17 | 6 | 0.167 | 0.100 | 0.333 | 0.353 | 0.466 | -0.0365943 | 3 | 0 |
| 19 | 6 | 0.167 | 0.117 | 0.206 | 0.138 | 0.217 | -0.0819392 | 1 | 0 |
| 23 | 6 | 0.167 | 0.117 | 0.431 | 0.396 | 0.557 | -0.0519441 | 1 | 0 |
| 27 | 6 | 0.222 | 0.133 | 0.328 | 0.414 | 0.458 | -0.0091199 | 3 | 0 |
| 29 | 6 | 0.111 | 0.067 | 0.180 | 0.502 | 0.539 | 0.000273934 | 2 | 0 |

## Frozen gates

- Tail recall: overall >= 0.80; every seed >= 0.667
- Tail Jaccard: overall >= 0.70; every seed >= 0.50
- Edge task Spearman: overall >= 0.50; every seed >= 0.35
- Normalized regret: median <= 0.10; P90 <= 0.25
- Observed-tail false-safe <= 0.01; true-safe recall >= 0.50

Evidence boundary: Selection uses only whole-seed training LOSO with a disjoint whole-seed safety calibration split. Development seeds 31 and 37 were not loaded. A passing result authorizes only registration of this frozen model and one development-gate evaluation; it does not authorize H=3 returns, held-out M24, X36, or an effect claim.
