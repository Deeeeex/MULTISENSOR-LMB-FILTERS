# V101 propagation-aware dwell: X36 t72 H=6

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Minimum dwell: `3` steps (gateway depth `2`)

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.581111 | -- | -- |
| V101 dwell | 80.700047 | +4.589% | +5.811% |

| t | Static | V101 | Gain |
|--:|--:|--:|--:|
| 72 | 86.118620 | 85.071354 | +1.216% |
| 73 | 85.408155 | 83.601644 | +2.115% |
| 74 | 86.384056 | 82.011795 | +5.061% |
| 75 | 85.605271 | 80.345140 | +6.145% |
| 76 | 82.342302 | 78.070034 | +5.188% |
| 77 | 81.628263 | 75.100317 | +7.997% |

- Dwell sets: `[1 2 4 5] -> [1 2 3 4 5] -> [1 2 3 4 5] -> [1 2 3 4 5 6] -> [1 2 3 4 5 6] -> [1 2 3 4 5 6]`
- Formation gains: `[2.265 2.866 5.512 8.367 8.758 0.1655]%`
- Minimum post-propagation gain: `+5.188%`
- Worst sensor / minimum formation: `+13.668% / +0.165%`
- Window / terminal consensus: `+7.670% / +11.632%`
- Static / V101 runtime: `185.43 / 180.34 s`
- Registered gate passed: `0`

## Evidence boundary

V101 is a topology-calibrated headroom probe on the matched static carrier graph. It applies a three-step minimum dwell to the causal V100 receiver-formation selections, because the registered X36 within-formation dominant tree has gateway depth two. The only changed execution page is t=76, where formations 5 and 6 remain protected. The schedule is frozen from observable V100 decisions before V101 tracking outcomes are opened. It tests the delayed-action mechanism and is not yet an online deployable policy or validation claim.
