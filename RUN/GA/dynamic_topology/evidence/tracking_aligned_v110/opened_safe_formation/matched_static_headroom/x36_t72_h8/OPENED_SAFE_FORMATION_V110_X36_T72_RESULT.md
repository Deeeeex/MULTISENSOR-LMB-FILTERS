# V110 opened safe-formation oracle: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V110 safe-formation oracle | 79.517797 | +5.378% | +3.479% |

| t | Static | Candidate | Gain | Protected | Released |
|--:|--:|--:|--:|:--|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [2 4 5] | 1 |
| 73 | 85.408155 | 83.601644 | +2.115% | [2 3 4 5] | 1 |
| 74 | 86.384056 | 82.701125 | +4.263% | [2 3 4 5] | 1 |
| 75 | 85.605271 | 81.720413 | +4.538% | [2 3 4 5] | [1 6] |
| 76 | 82.342302 | 78.795026 | +4.308% | [2 3 4 5] | [1 6] |
| 77 | 81.628263 | 74.666625 | +8.528% | [2 3 4 5] | [1 6] |
| 78 | 83.556650 | 76.244124 | +8.752% | [2 3 4 5] | [1 6] |
| 79 | 81.253892 | 73.342063 | +9.737% | [2 3 4 5] | [1 6] |

- Formation gains: `[-0.2158 4.805 7.711 8.97 11.25 6.859e-05]%`
- Formation-by-time gains (rows F1--F6):

```text
[0 0 0 0.003988 -0.9127 -1.356 -1.167 1.826;2.578 1.533 5.969 6.172 -4.928 5.07 10.27 11.14;0 3.123 4.065 4.628 12.47 10.38 12.86 15.17;1.01 2.944 7.934 9.996 8.744 19.14 11.87 9.873;3.686 5.659 7.723 6.413 11.37 17.08 17.84 18.6;0 0 0 0 0 0 0 0.0005866]
```
- Minimum formation-time gain: `-4.928%`
- Minimum after maturity: `+4.308%`
- F6 non-gateway terminal gain: `+0.000%`
- Worst sensor / minimum formation: `+14.912% / -0.216%`
- Window / terminal consensus: `+7.619% / +17.980%`
- Static / candidate runtime: `251.51 / 246.85 s`
- Registered gate passed: `0`

## Evidence boundary

V110 is a frozen retrospective action-space oracle. It keeps the matched static route and V109 explicit source-abstention semantics, but protects only F2--F5 because opened V105 outcomes identify F1 and F6 as the nonpositive formations. The complete F1/F6 outcomes are therefore used to define the action before this V110 rollout. Measurements, delivery uniforms, filter RNG, communication model and the frozen static full-payload outcome remain paired. V110 is not deployable and cannot support validation or generalization claims; it tests whether a perfect formation-risk gate has enough headroom to retain the aggregate gain without local harm.
