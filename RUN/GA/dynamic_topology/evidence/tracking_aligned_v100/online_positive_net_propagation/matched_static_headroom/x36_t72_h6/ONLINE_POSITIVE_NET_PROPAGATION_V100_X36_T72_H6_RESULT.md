# V100 online positive-net propagation: X36 t72 H=6

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Formation-graph directed diameter: `5`
- Horizon: `one action step + diameter = 6`

| Arm | Mean E-OSPA | Gain vs static | Gain vs fixed |
|:--|--:|--:|--:|
| Static full payload | 84.581111 | -- | -- |
| Fixed initial set | 81.443184 | +3.710% | -- |
| Online V100 | 80.807326 | +4.462% | +0.781% |

| t | Static | Fixed | Online | Online/static |
|--:|--:|--:|--:|--:|
| 72 | 86.118620 | 85.071354 | 85.071354 | +1.216% |
| 73 | 85.408155 | 84.028817 | 83.601644 | +2.115% |
| 74 | 86.384056 | 82.590310 | 82.011795 | +5.061% |
| 75 | 85.605271 | 80.922046 | 80.345140 | +6.145% |
| 76 | 82.342302 | 79.571234 | 78.535347 | +4.623% |
| 77 | 81.628263 | 76.475345 | 75.278676 | +7.779% |

- Online sets: `[1 2 4 5] -> [1 2 3 4 5] -> [1 2 3 4 5] -> [1 2 3 4 5 6] -> [1 2 3 4] -> [1 2 3 4 5 6]`
- Minimum post-propagation gain: `+4.623%`
- Worst sensor / minimum formation: `+13.668% / +0.165%`
- Window / terminal consensus: `+7.376% / +11.377%`
- Attempted-byte saving: `+5.250%`
- Static / fixed / online runtime: `185.76 / 182.66 / 1015.21 s`
- H=6 propagation gate passed: `0`

## Evidence boundary

V100 retains the exact V99 spatial selector and matched static carrier graph, but evaluates six fusion steps: one decision step plus the registered X36 formation-graph directed diameter of five. Static, fixed-set and online arms share the cached posterior, measurements, link uniforms, filter RNG, fusion weights and communication constraints. The probe tests whether the V99 gain is propagation-delayed or transient; it does not tune an outcome-based duration and remains opened development evidence only.
