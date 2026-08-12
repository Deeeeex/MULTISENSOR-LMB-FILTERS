# V130 sparse posterior correction: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V130 sparse posterior correction | 79.760824 | +5.089% | +6.115% |

| t | Static | Candidate | Gain | Protected formations |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] |
| 74 | 86.384056 | 82.011795 | +5.061% | [1 2 3 4 5] |
| 75 | 85.605271 | 80.345140 | +6.145% | [1 2 3 4 5 6] |
| 76 | 82.342302 | 78.349919 | +4.849% | [1 2 3 4 5 6] |
| 77 | 81.628263 | 75.962743 | +6.941% | [1 2 3 4 5 6] |
| 78 | 83.556650 | 77.784044 | +6.909% | [1 2 3 4 5 6] |
| 79 | 81.253892 | 74.959954 | +7.746% | [1 2 3 4 5 6] |

- Formation gains: `[-0.8777 3.806 7.711 8.97 11.25 -0.02109]%`
- Minimum after maturity: `+4.849%`
- F6 non-gateway terminal gain: `-2.940%`
- Worst sensor / minimum formation: `+14.841% / -0.878%`
- Minimum formation-time gain: `-11.556%`
- Window / terminal consensus: `+8.825% / +19.545%`
- Static / candidate runtime: `251.51 / 248.49 s`
- Auxiliary-state maintenance cost included: `1`
- Protected-input light weight factor: `0.500`
- Light-correction formations by page: `[] | [] | [] | [] | [] | 1 | [1 6] | [1 6]`
- Full-correction formations by page: `[] | [] | [] | [] | 2 | [] | [] | []`
- Auxiliary runtime included / memory quantified: `1 / 1`
- Anchor-maintained nodes by time: `[0 0 0 0 0 0 0 0]`
- Registered gate passed: `0`

## Evidence boundary

V130 is a paired X36 seed-211 t=72 H=8 sparse-action upper bound. It keeps the V105 control-only payload as the default, applies a 0.5-weight moment-compressed posterior to F1 on pages 6--8 and F6 on pages 7--8, and restores the full posterior to F2 on page 5. The formation-page correction support is inherited from the opened V126 failure mask and is therefore privileged; target truth and future measurements do not enter message construction itself. Actual control, light and full payload bytes are included and no parallel state is maintained. V130 tests hybrid action-space headroom only and is not deployable, validation, or generalization evidence.
