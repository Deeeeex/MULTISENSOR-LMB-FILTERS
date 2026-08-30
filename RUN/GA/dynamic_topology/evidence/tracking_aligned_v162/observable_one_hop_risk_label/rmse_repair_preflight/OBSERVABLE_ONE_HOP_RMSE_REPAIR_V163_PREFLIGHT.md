# V163 observable F3/F5 RMSE-repair preflight

- Preset / seed: `x36-formation-fov / 211`
- Privileged repair pages / times: `[5 7 8] / [76 78 79]`
- Repair formations: `3 | [3 5] | [3 5]`
- Receiver-time cells: `30`

| t | F | Sensor | Labels | E-OSPA gain | RMSE gain | Bytes | Snapshot mismatch |
|--:|--:|--:|--:|--:|--:|--:|--:|
| 76 | 3 | 13 | 4 | +17.758126 | +2.625238 | 20576 | +0.000000 |
| 76 | 3 | 14 | 4 | +1.018659 | +3.622492 | 20584 | +0.000000 |
| 76 | 3 | 15 | 4 | +24.678881 | +134.456278 | 20584 | +0.000000 |
| 76 | 3 | 16 | 4 | +17.814939 | +2.439732 | 20584 | +0.000000 |
| 76 | 3 | 17 | 4 | +16.073052 | -5.107760 | 20632 | +0.000000 |
| 76 | 3 | 18 | 4 | +8.835055 | +4.107340 | 18560 | +0.000000 |
| 78 | 3 | 13 | 4 | +9.196023 | +5.724042 | 20712 | +0.000000 |
| 78 | 3 | 14 | 4 | +15.707521 | +4.438920 | 20712 | +0.000000 |
| 78 | 3 | 15 | 4 | +25.959650 | +149.188026 | 20712 | +0.000000 |
| 78 | 3 | 16 | 4 | +25.962082 | +9.431446 | 20712 | +0.000000 |
| 78 | 3 | 17 | 4 | +26.015577 | +9.690585 | 20712 | +0.000000 |
| 78 | 3 | 18 | 4 | +25.368444 | +6.864363 | 20712 | +0.000000 |
| 78 | 5 | 25 | 4 | +22.510445 | -11.275118 | 20632 | +0.000000 |
| 78 | 5 | 26 | 4 | +23.018008 | -5.339322 | 20680 | +0.000000 |
| 78 | 5 | 27 | 4 | +22.932889 | -9.315494 | 20680 | +0.000000 |
| 78 | 5 | 28 | 4 | +23.033345 | -6.066783 | 20680 | +0.000000 |
| 78 | 5 | 29 | 4 | +23.141174 | -5.246940 | 20680 | +0.000000 |
| 78 | 5 | 30 | 4 | +22.183541 | -8.232326 | 20680 | +0.000000 |
| 79 | 3 | 13 | 4 | +16.194610 | +6.400529 | 20704 | +0.000000 |
| 79 | 3 | 14 | 4 | +25.881353 | +8.138529 | 20704 | +0.000000 |
| 79 | 3 | 15 | 4 | +26.240262 | +9.646335 | 20704 | +0.000000 |
| 79 | 3 | 16 | 4 | +25.875772 | +153.454696 | 20704 | +0.000000 |
| 79 | 3 | 17 | 4 | +26.225794 | +9.616894 | 20704 | +0.000000 |
| 79 | 3 | 18 | 4 | +25.874480 | +8.132874 | 20704 | +0.000000 |
| 79 | 5 | 25 | 4 | +24.391860 | -0.255724 | 20720 | +0.000000 |
| 79 | 5 | 26 | 4 | +23.479416 | -4.068556 | 20720 | +0.000000 |
| 79 | 5 | 27 | 4 | +23.415441 | -3.892615 | 20720 | +0.000000 |
| 79 | 5 | 28 | 4 | +23.440426 | -4.458993 | 20720 | +0.000000 |
| 79 | 5 | 29 | 4 | +23.425948 | -4.131249 | 20720 | +0.000000 |
| 79 | 5 | 30 | 4 | +23.511060 | -4.174695 | 20720 | +0.000000 |

- Total immediate E-OSPA / RMSE gain: `639.163832 / 456.412742`
- Harmful E-OSPA / RMSE cells: `0 / 13`
- Minimum cell E-OSPA / RMSE gain: `+1.018659 / -11.275118`
- Mean / max absolute snapshot mismatch: `0.000000 / 0.000000`
- Additional attempted bytes: `618368`
- Projected byte saving: `+0.295%`
- Base formation RMSE: `[35.8461 25.5221 32.0804 162.075 12.638 63.1533]`
- Projected formation RMSE: `[35.8461 25.5221 21.1872 162.075 14.0225 63.1533]`
- Projected formation RMSE gain: `[25.846 5.7725 14.275 1.789 -13.96 23.294]%`
- Mechanism gate passed: `0`

## F5 joint-positive truth mechanism oracle

| t | Sensor | Actions | E-OSPA gain | RMSE gain | Bytes |
|--:|--:|--:|--:|--:|--:|
| 78 | 25 | 4 | +0.299253 | +3.699788 | 20728 |
| 78 | 26 | 4 | +0.391604 | +4.103672 | 20728 |
| 78 | 27 | 4 | +0.329798 | +3.599130 | 20728 |
| 78 | 28 | 4 | +0.368799 | +3.823401 | 20728 |
| 78 | 29 | 4 | +0.455679 | +4.523299 | 20728 |
| 78 | 30 | 4 | +0.391830 | +4.073309 | 20728 |
| 79 | 25 | 4 | +0.395732 | +4.931591 | 20768 |
| 79 | 26 | 4 | +0.404257 | +4.519232 | 20720 |
| 79 | 27 | 4 | +0.391413 | +4.474118 | 20768 |
| 79 | 28 | 4 | +0.380186 | +3.942147 | 20720 |
| 79 | 29 | 4 | +0.434483 | +4.196629 | 20720 |
| 79 | 30 | 4 | +0.388451 | +3.929984 | 20720 |

- Joint-oracle actions: `48`
- Joint-oracle F5 E-OSPA / RMSE gain: `4.631484 / 49.816299`
- F3 analytic + F5 oracle additional bytes: `618800`
- Projected byte saving: `+0.293%`
- Projected formation RMSE gain: `[25.846 5.7725 14.275 1.789 5.7265 23.294]%`
- Joint-oracle mechanism gate passed: `1`

## Evidence boundary

V163 is an opened, nonrecursive repair-mechanism screen. The F3/F5 receiver-time schedule is selected after observing the V162 formation-RMSE failure and is privileged. The minimum-risk source and positive-risk Top-4 label rules remain truth-free. Edits use pre-action V157 fused snapshots and all-delivered physical links; truth scores immediate E-OSPA and matched-position RMSE only. The projected recursive formation means add these snapshot deltas to the V162 outcome, so snapshot mismatch is reported explicitly. A separate F5 joint-positive truth oracle asks whether any one-hop source-label replacement can improve both metrics; it is a mechanism upper bound and does not define the deployable selector. A positive preflight only authorizes a recursive mechanism probe; it is not an online trigger, tracking result or validation claim.
