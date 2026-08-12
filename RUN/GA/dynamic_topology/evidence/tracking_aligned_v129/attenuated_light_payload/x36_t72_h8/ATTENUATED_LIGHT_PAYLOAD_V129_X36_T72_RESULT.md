# V129 attenuated light payload: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V129 attenuated light payload | 82.680873 | +1.614% | +7.865% |

| t | Static | Candidate | Gain | Protected formations |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.697182 | +0.489% | [1 2 4 5] |
| 73 | 85.408155 | 85.112131 | +0.347% | [1 2 3 4 5] |
| 74 | 86.384056 | 85.307627 | +1.246% | [1 2 3 4 5] |
| 75 | 85.605271 | 84.110113 | +1.747% | [1 2 3 4 5 6] |
| 76 | 82.342302 | 82.226066 | +0.141% | [1 2 3 4 5 6] |
| 77 | 81.628263 | 79.078776 | +3.123% | [1 2 3 4 5 6] |
| 78 | 83.556650 | 81.037490 | +3.015% | [1 2 3 4 5 6] |
| 79 | 81.253892 | 78.877600 | +2.925% | [1 2 3 4 5 6] |

- Formation gains: `[0.5487 -0.5593 2.76 4.311 2.522 0.2617]%`
- Minimum after maturity: `+0.141%`
- F6 non-gateway terminal gain: `+0.023%`
- Worst sensor / minimum formation: `+6.982% / -0.559%`
- Minimum formation-time gain: `-13.826%`
- Window / terminal consensus: `+2.878% / +6.404%`
- Static / candidate runtime: `251.51 / 245.94 s`
- Auxiliary-state maintenance cost included: `1`
- Protected-input light weight factor: `0.500`
- Auxiliary runtime included / memory quantified: `1 / 1`
- Anchor-maintained nodes by time: `[0 0 0 0 0 0 0 0]`
- Registered gate passed: `0`

## Evidence boundary

V129 is a paired X36 seed-211 t=72 H=8 soft-protection attribution. It retains the V105 static carrier, fusion weights and observable formation schedule, but replaces each protected control-only message with the sender's current per-label moment-compressed LMB posterior. The protected neighbor input uses a frozen 0.5 reliability factor before normalization. The factor is a single registered midpoint between V105 abstention and full static fusion, not an outcome-selected sweep. No parallel filter, target truth, future measurement or opened V126 rollback mask enters the candidate. Actual compressed payload bytes are included. This first opened case tests the mechanism and is not validation or a generalization claim.
