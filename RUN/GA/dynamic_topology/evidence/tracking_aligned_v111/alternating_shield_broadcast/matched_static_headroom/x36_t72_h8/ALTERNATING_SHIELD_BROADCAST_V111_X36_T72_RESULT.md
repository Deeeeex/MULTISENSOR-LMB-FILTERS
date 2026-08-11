# V111 alternating shield/broadcast: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V111 alternating shield/broadcast | 79.663569 | +5.204% | +5.483% |

| t | Static | Candidate | Gain | Protected | Broadcast |
|--:|--:|--:|--:|:--|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] | [] |
| 73 | 85.408155 | 82.330473 | +3.603% | [1 2 3 4 5] | [1 2 4 5] |
| 74 | 86.384056 | 82.017574 | +5.055% | [1 2 3 4 5] | [] |
| 75 | 85.605271 | 80.498531 | +5.965% | [1 2 3 4 5 6] | [1 2 3 4 5] |
| 76 | 82.342302 | 77.744767 | +5.583% | [1 2 3 4 5 6] | [] |
| 77 | 81.628263 | 76.740866 | +5.987% | [1 2 3 4 5 6] | [1 2 3 4 5 6] |
| 78 | 83.556650 | 77.470765 | +7.284% | [1 2 3 4 5 6] | [] |
| 79 | 81.253892 | 75.434219 | +7.162% | [1 2 3 4 5 6] | [1 2 3 4 5 6] |

- Formation gains: `[-0.8158 5.394 7.449 7.853 11.75 -0.1541]%`
- Formation-by-time gains (rows F1--F6):

```text
[0 1.131 4.806 9.174 4.156 -5.944 -8.609 -15.79;2.578 5.489 5.957 5.224 -3.791 3.1 10.3 13.28;0 3.123 4.065 0.6442 14.06 10.55 12.87 15.23;1.01 6.048 7.91 9.744 8.693 9.866 10.81 8.533;3.686 5.669 7.75 10.5 11.35 17.06 17.83 18.6;0 0 -0.0002317 0.001468 0.001683 0.9767 -1.055 -1.257]
```
- Minimum formation-time gain: `-15.793%`
- Minimum after maturity: `+5.583%`
- F6 non-gateway terminal gain: `-2.955%`
- Worst sensor / minimum formation: `+16.734% / -0.816%`
- Window / terminal consensus: `+10.685% / +20.684%`
- Static / candidate runtime: `251.51 / 241.06 s`
- Registered gate passed: `0`

## Evidence boundary

V111 is a frozen H=8 propagation-control headroom probe. It extends the V102 one-step-delayed alternating shield/broadcast cadence to the complete V105 H=8 protection schedule and uses explicit abstention for protected cross inputs. Broadcast and reference recovery pages alternate, every route page remains physically reachable, and the matched static full-payload outcome is reused. The schedule is constructed without V110 outcomes, but it remains frozen development evidence rather than an online policy or a validation/generalization claim.
