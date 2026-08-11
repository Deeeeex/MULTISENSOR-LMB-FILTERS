# V109 explicit label abstention: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V109 explicit abstention | 79.617863 | +5.259% | +6.117% |

| t | Static | Candidate | Gain | Protected | Released |
|--:|--:|--:|--:|:--|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] | [] |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] | [] |
| 74 | 86.384056 | 82.011795 | +5.061% | [1 2 3 4 5] | [] |
| 75 | 85.605271 | 80.345140 | +6.145% | [1 2 3 4 5 6] | [] |
| 76 | 82.342302 | 78.070034 | +5.188% | [1 2 3 4 5 6] | [] |
| 77 | 81.628263 | 75.100317 | +7.997% | [1 2 3 4 5 6] | [] |
| 78 | 83.556650 | 77.174602 | +7.638% | [1 2 3 4 5 6] | [] |
| 79 | 81.253892 | 75.568021 | +6.998% | [1 2 3 4 5 6] | [] |

- Formation gains: `[-0.9312 4.805 7.711 8.97 11.25 -0.0212]%`
- Formation-by-time gains (rows F1--F6):

```text
[0 0 4.833 9.205 4.202 -5.861 -8.61 -15.75;2.578 1.533 5.969 6.172 -4.928 5.07 10.27 11.14;0 3.123 4.065 4.628 12.47 10.38 12.86 15.17;1.01 2.944 7.934 9.996 8.744 19.14 11.87 9.873;3.686 5.659 7.723 6.413 11.37 17.08 17.84 18.6;0 0 0 0.001593 0.001866 0.9796 -0.008509 -1.245]
```
- Minimum formation-time gain: `-15.754%`
- Minimum after maturity: `+5.188%`
- F6 non-gateway terminal gain: `-2.940%`
- Worst sensor / minimum formation: `+16.734% / -0.931%`
- Window / terminal consensus: `+9.650% / +17.214%`
- Static / candidate runtime: `251.51 / 242.09 s`
- Registered gate passed: `0`

## Evidence boundary

V109 is a frozen H=8 fusion-semantics attribution. It keeps the matched static fixed-counter-clockwise route, V105 formation schedule, control synopsis bytes, link attempts, delivery uniforms and filter RNG. The only change is that a delivered selective empty payload abstains from fusion instead of participating as a fov-aware censored missing-label source. V109 uses no new truth or future outcome to choose actions. It tests whether V105's gain and local harm come from source abstention or bulk low-existence evidence. It is development evidence only and cannot support validation or generalization claims.
