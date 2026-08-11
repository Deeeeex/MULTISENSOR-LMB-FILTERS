# V107 early protection-release oracle: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V107 early-release oracle | 79.684870 | +5.179% | +4.848% |

| t | Static | Candidate | Gain | Protected | Released |
|--:|--:|--:|--:|:--|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] | [] |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] | [] |
| 74 | 86.384056 | 82.011795 | +5.061% | [1 2 3 4 5] | [] |
| 75 | 85.605271 | 80.345140 | +6.145% | [1 2 3 4 5 6] | [] |
| 76 | 82.342302 | 78.070034 | +5.188% | [2 3 4 5 6] | 1 |
| 77 | 81.628263 | 76.054376 | +6.828% | [2 3 4 5] | [1 6] |
| 78 | 83.556650 | 77.471863 | +7.282% | [2 3 4 5] | [1 6] |
| 79 | 81.253892 | 74.852752 | +7.878% | [2 3 4 5] | [1 6] |

- Formation gains: `[-0.8652 4.805 7.711 8.97 11.25 -0.551]%`
- Formation-by-time gains (rows F1--F6):

```text
[0 0 4.833 9.205 4.202 -12.18 -11 -5.681;2.578 1.533 5.969 6.172 -4.928 5.07 10.27 11.14;0 3.123 4.065 4.628 12.47 10.38 12.86 15.17;1.01 2.944 7.934 9.996 8.744 19.14 11.87 9.873;3.686 5.659 7.723 6.413 11.37 17.08 17.84 18.6;0 0 0 0.001593 0.001866 -0.007679 3.25e-05 -4.708]
```
- Minimum formation-time gain: `-12.181%`
- Minimum after maturity: `+5.188%`
- F6 non-gateway terminal gain: `-5.752%`
- Worst sensor / minimum formation: `+16.569% / -0.865%`
- Window / terminal consensus: `+8.818% / +17.288%`
- Static / candidate runtime: `251.51 / 245.61 s`
- Registered gate passed: `0`

## Evidence boundary

V107 is a frozen retrospective one-page-early release oracle. It starts from V105 but releases F1 at t=76 and F6 at t=77, one page before the opened V105 formation or peer outcomes first reverse sign. Truth and opened V105 outcomes define these pages, so V107 is not deployable and cannot support validation or generalization claims. Every topology adjacency and fusion-weight row remains the matched static fixed-counter-clockwise route and no handoff occurs. The frozen H=8 static outcome is reused only after preset, seed, receiver mode, horizon, cache path and cache SHA-256 match. V107 distinguishes a late reactive release from a structural failure of binary formation-level full/control-only switching.
