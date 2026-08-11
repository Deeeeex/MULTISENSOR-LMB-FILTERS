# V108 signed complete-label exception oracle: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`
- Oracle design uses truth/future opened states: `1 / 1`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V108 signed-label oracle | 79.636297 | +5.237% | +5.870% |

| t | Static | Candidate | Gain | Protected | Labels sent |
|--:|--:|--:|--:|:--|--:|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] | 0 |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] | 0 |
| 74 | 86.384056 | 82.011696 | +5.062% | [1 2 3 4 5] | 3 |
| 75 | 85.605271 | 80.345003 | +6.145% | [1 2 3 4 5 6] | 6 |
| 76 | 82.342302 | 78.069592 | +5.189% | [1 2 3 4 5 6] | 3 |
| 77 | 81.628263 | 75.100166 | +7.997% | [1 2 3 4 5 6] | 6 |
| 78 | 83.556650 | 77.327502 | +7.455% | [1 2 3 4 5 6] | 3 |
| 79 | 81.253892 | 75.563423 | +7.003% | [1 2 3 4 5 6] | 3 |

- Formation gains: `[-0.9303 4.805 7.711 8.97 11.25 -0.1506]%`
- Formation-by-time gains (rows F1--F6):

```text
[0 0 4.833 9.207 4.204 -5.86 -8.608 -15.75;2.578 1.533 5.969 6.172 -4.928 5.07 10.27 11.14;0 3.123 4.065 4.628 12.47 10.38 12.86 15.17;1.01 2.944 7.934 9.996 8.744 19.14 11.87 9.873;3.686 5.659 7.723 6.413 11.37 17.08 17.84 18.6;0 0 0 0.00132 0.003679 0.9798 -1.076 -1.211]
```
- Minimum formation-time gain: `-15.753%`
- Minimum after maturity: `+5.189%`
- F6 non-gateway terminal gain: `-2.904%`
- Worst sensor / minimum formation: `+16.734% / -0.930%`
- Window / terminal consensus: `+9.643% / +17.658%`
- Static / candidate runtime: `251.51 / 248.78 s`
- Registered gate passed: `0`

## Evidence boundary

V108 is a frozen retrospective action-space headroom oracle. It keeps the exact V105 static route and control-only formation schedule but admits at most three complete Bernoulli Gaussian-mixture labels on an actually delivered F1 or F6 gateway edge. The label schedule was selected from opened V105 local posterior snapshots and target truth using positive one-round capped expected-risk reduction under the repository's componentwise powered-GM KLA approximation. It therefore uses truth and future opened states, is not deployable, and cannot support validation or generalization claims. Its sole purpose is to test whether sparse complete-label exceptions have enough headroom to repair V105's F1/F6 losses before a truth-free estimator or GNN is designed.
