# V132 risk-formation light network anchor: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V132 risk-formation light network anchor | 78.809058 | +6.221% | +7.309% |

| t | Static | Candidate | Gain | Protected formations |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] |
| 74 | 86.384056 | 82.011795 | +5.061% | [1 2 3 4 5] |
| 75 | 85.605271 | 80.345140 | +6.145% | [1 2 3 4 5 6] |
| 76 | 82.342302 | 78.227371 | +4.997% | [1 2 3 4 5 6] |
| 77 | 81.628263 | 75.003922 | +8.115% | [1 2 3 4 5 6] |
| 78 | 83.556650 | 73.645380 | +11.862% | [1 2 3 4 5 6] |
| 79 | 81.253892 | 72.565858 | +10.692% | [1 2 3 4 5 6] |

- Formation gains: `[2.855 6.267 7.711 8.97 11.25 0.5061]%`
- Minimum after maturity: `+4.997%`
- F6 non-gateway terminal gain: `-2.968%`
- Worst sensor / minimum formation: `+17.849% / +0.506%`
- Minimum formation-time gain: `-6.111%`
- Window / terminal consensus: `+12.652% / +30.318%`
- Static / candidate runtime: `251.51 / 314.13 s`
- Auxiliary-state maintenance cost included: `0`
- Additional posterior payload messages / bytes: `240 / 1040640`
- Anchor attempted bytes by time: `[128352 129888 127584 125856 128544 132768 132000 135648]`
- Anchor delivered bytes by time: `[115440 121088 127584 125856 119744 124160 127696 131152]`
- Auxiliary runtime included / memory quantified: `1 / 0`
- Anchor-maintained nodes by time: `[36 36 36 36 36 36 36 36]`
- Registered gate passed: `0`

## Evidence boundary

V132 is a paired X36 seed-211 t=72 H=8 message-reallocation mechanism test. It retains the V131 parallel moment-compressed network anchor and reallocates auxiliary directed messages on the receiver rows of opened risk formations F1, F2 and F6 at every page instead of refreshing all six formations on alternating pages. The working V105 route, payload and protection schedule remain unchanged. Actual auxiliary attempted/delivered bytes and runtime are charged, while extra anchor memory remains unquantified. The receiver formation set and rollback cells use opened V126/V131 outcomes, so V132 is privileged development evidence only and cannot support online, validation, or generalization claims.
