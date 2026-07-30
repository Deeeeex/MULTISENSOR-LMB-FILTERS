# M24 certified-overlap one-shot held-out validation

- Contract: `m24-adaptive-dominant-certified-overlap-heldout-v1`
- Generated: 2026-07-31 05:24:59
- Run commit: `605c9d82f01feb19a1dd49aea0d283bdbf99f4db`
- Margin: `2.0%`
- Seeds: `[41 43 47 53 59]`
- Window: `[75 76 77 78 79 80 81 82 83]`

| Seed | CCW E-OSPA | Candidate E-OSPA | Gain | Byte delta | Msgs | Maps | Safe |
|--:|--:|--:|--:|--:|--:|--:|:--:|
| 41 | 11.634356 | 5.558843 | +52.220% | -6.933% | 36.56 | 9 | 1 |
| 43 | 15.011374 | 5.598992 | +62.702% | -8.785% | 36.44 | 9 | 1 |
| 47 | 16.488875 | 11.414642 | +30.774% | -8.647% | 36.22 | 9 | 1 |
| 53 | 9.532449 | 6.314228 | +33.761% | -7.902% | 36.56 | 9 | 1 |
| 59 | 7.901186 | 5.314133 | +32.743% | -8.440% | 36.44 | 9 | 1 |

## Preregistered held-out gate

- Mean E-OSPA gain: `+43.533%`
- Positive-seed fraction: `1.000`
- Aggregate worst-node gain: `+45.014%`
- Aggregate consensus gain: `+39.137%`
- Aggregate attempted-byte saving: `+8.144%`
- Maximum positive byte delta: `+0.000%`
- Required minimum mean gain: `5.000%`
- Required positive-seed fraction: `0.800`
- Required minimum byte saving: `2.000%`
- Hard safety passed: `1`
- Tracking gate passed: `1`
- Tail gate passed: `1`
- Communication gate passed: `1`
- Held-out gate passed: `1`
- Fresh M24 validation claim allowed: `1`
- Next: `freeze-heldout-M24-evidence-and-preregister-X36-scale-validation`

These five M24 seeds were locked before v4 method selection and were opened once only after the candidate, margin and gates were frozen. The evidence is a paired conditional continuation over t=75:83 from a common cached prefix, not a full-episode estimate. X36 remains unopened in this report.
