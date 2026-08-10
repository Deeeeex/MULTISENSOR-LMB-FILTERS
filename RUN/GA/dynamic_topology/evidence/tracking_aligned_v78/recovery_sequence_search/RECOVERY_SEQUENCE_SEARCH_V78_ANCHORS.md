# V78 recovery-sequence search

- Schedules: `CRR / CRC / CCR / CCC`
- Candidate RIE rechecked before every C round: `1`
- Tracking outcome label available: `0`

## m24-formation-fov-merge-split / t=80

- Direct-safe formations: `3`

### Historical exact V71/V72 route

- Applied slot triples: `[14 10 21;17 19 3]`

| Schedule | Centered energy by round | Common energy by round | Peak factor | Terminal factor | RIE eligible | Monotone |
|:--|:--|:--|--:|--:|:--:|:--:|
| CRR | `[0.00503634 0.00557678 0.00405308]` | `[0.000124346 0.000184253 0.000824323]` | 1.107308 | 0.804767 | 1 | 0 |
| CRC | `[0.00503634 0.00557678 0.00661927]` | `[0.000124346 0.000184253 0.00134394]` | 1.314301 | 1.314301 | 1 | 0 |
| CCR | `[0.00503634 0.00999893 0.00674626]` | `[0.000124346 0.000459157 0.00133395]` | 1.985355 | 1.339516 | 0 | 0 |
| CCC | `[0.00503634 0.00999893 0.00976105]` | `[0.000124346 0.000459157 0.00208242]` | 1.985355 | 1.938123 | 0 | 0 |

- Selected schedule: `CRR`
- Selected peak / terminal factor: `1.107308 / 0.804767`
- Selected monotone / any monotone eligible: `0 / 0`

### Prospective exact V73 route

- Applied slot triples: `[14 10 21;17 19 20]`

| Schedule | Centered energy by round | Common energy by round | Peak factor | Terminal factor | RIE eligible | Monotone |
|:--|:--|:--|--:|--:|:--:|:--:|
| CRR | `[0.00400088 0.00421286 0.00416493]` | `[0.00017126 0.000342063 0.00086866]` | 1.052982 | 1.041003 | 1 | 0 |
| CRC | `[0.00400088 0.00421286 0.00663845]` | `[0.00017126 0.000342063 0.00143938]` | 1.659246 | 1.659246 | 1 | 0 |
| CCR | `[0.00400088 0.00749824 0.00695482]` | `[0.00017126 0.000762922 0.00144719]` | 1.874146 | 1.738322 | 0 | 0 |
| CCC | `[0.00400088 0.00749824 0.00976737]` | `[0.00017126 0.000762922 0.00226203]` | 2.441305 | 2.441305 | 0 | 0 |

- Selected schedule: `CRR`
- Selected peak / terminal factor: `1.052982 / 1.041003`
- Selected monotone / any monotone eligible: `0 / 0`

## x36-formation-fov-merge-split / t=52

- Direct-safe formations: `4`

### Historical exact V71/V72 route

- Applied slot triples: `[20 16 34;23 25 35]`

| Schedule | Centered energy by round | Common energy by round | Peak factor | Terminal factor | RIE eligible | Monotone |
|:--|:--|:--|--:|--:|:--:|:--:|
| CRR | `[0.00179825 0.00144706 0.0028974]` | `[5.163e-05 5.73559e-05 0.000378331]` | 1.611232 | 1.611232 | 1 | 0 |
| CRC | `[0.00179825 0.00144706 0.00442965]` | `[5.163e-05 5.73559e-05 0.000624668]` | 2.463305 | 2.463305 | 1 | 0 |
| CCR | `[0.00179825 0.0032641 0.00431457]` | `[5.163e-05 0.00018582 0.000657775]` | 2.399309 | 2.399309 | 1 | 0 |
| CCC | `[0.00179825 0.0032641 0.00627488]` | `[5.163e-05 0.00018582 0.00105549]` | 3.489431 | 3.489431 | 1 | 0 |

- Selected schedule: `CRR`
- Selected peak / terminal factor: `1.611232 / 1.611232`
- Selected monotone / any monotone eligible: `0 / 0`

### Prospective exact V73 route

- Applied slot triples: `[20 16 34;23 25 36]`

| Schedule | Centered energy by round | Common energy by round | Peak factor | Terminal factor | RIE eligible | Monotone |
|:--|:--|:--|--:|--:|:--:|:--:|
| CRR | `[0.00198748 0.0014756 0.00289954]` | `[5.74972e-05 5.96174e-05 0.000378453]` | 1.458899 | 1.458899 | 1 | 0 |
| CRC | `[0.00198748 0.0014756 0.00444037]` | `[5.74972e-05 5.96174e-05 0.000625176]` | 2.234167 | 2.234167 | 1 | 0 |
| CCR | `[0.00198748 0.00327906 0.00433351]` | `[5.74972e-05 0.00018789 0.000658521]` | 2.180401 | 2.180401 | 1 | 0 |
| CCC | `[0.00198748 0.00327906 0.00628317]` | `[5.74972e-05 0.00018789 0.00105607]` | 3.161371 | 3.161371 | 1 | 0 |

- Selected schedule: `CRR`
- Selected peak / terminal factor: `1.458899 / 1.458899`
- Selected monotone / any monotone eligible: `0 / 0`

## Summary

- Any / all historical cases have a monotone schedule: `0 / 0`
- Any / all aligned cases have a monotone schedule: `0 / 0`
- Route executed / tracking outcome read: `0 / 0`
- Validation claim allowed: `0`

## Evidence boundary

V78 keeps the V75 direct-safe first-round pulse fixed and enumerates reference versus the same candidate route in each of two source-only recovery rounds: CRR, CRC, CCR, and CCC. Before every candidate round the frozen V75 replacement-innovation gate is recomputed on the current virtual posterior; a failed round makes the schedule ineligible. Eligible schedules are ranked by peak centered-energy expansion after the first round, terminal centered factor, and candidate recovery-round count. The common mode is unconstrained. Fusion uses formal mixture-aware heavy receivers, registered directed weights, and deterministic current-link reliability. No packet draw, prediction, target motion, new measurement, truth, future link page, route execution, tracking outcome, or model training is used.
