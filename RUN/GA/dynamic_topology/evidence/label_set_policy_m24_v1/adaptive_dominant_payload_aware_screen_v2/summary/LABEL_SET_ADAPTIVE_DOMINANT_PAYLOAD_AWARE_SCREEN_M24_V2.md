# M24 payload-aware adaptive-dominant screen

- Contract: `label-set-safe-adaptive-dominant-payload-budget-v2`
- Generated: 2026-07-31 02:41:24
- Run commit: `0a12e7641981e1dfd02954f8bf952ed87ac03988`
- Seeds: `[11 19 23]`
- Window: `[75 76 77 78 79 80 81 82 83]`

| Margin | Seed | CCW E-OSPA | Candidate E-OSPA | Gain | CCW worst | Candidate worst | CCW consensus | Candidate consensus | Byte delta | Maps | Safe |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| 0.0% | 11 | 19.732356 | 12.011647 | +39.127% | 48.941010 | 28.469563 | 24.800230 | 17.623245 | -0.296% | 9 | 1 |
| 0.0% | 19 | 20.813255 | 17.879595 | +14.095% | 36.195719 | 33.204451 | 23.528471 | 21.520774 | +0.251% | 9 | 1 |
| 0.0% | 23 | 7.369449 | 6.111220 | +17.074% | 24.487945 | 14.806564 | 10.146024 | 8.427584 | -0.465% | 9 | 1 |
| 2.5% | 11 | 19.732356 | 14.849145 | +24.747% | 48.941010 | 28.414215 | 24.800230 | 21.051905 | -0.082% | 9 | 1 |
| 2.5% | 19 | 20.813255 | 18.506609 | +11.083% | 36.195719 | 32.134478 | 23.528471 | 22.341813 | -0.505% | 9 | 1 |
| 2.5% | 23 | 7.369449 | 5.590800 | +24.135% | 24.487945 | 13.226840 | 10.146024 | 7.609309 | -2.189% | 9 | 1 |
| 5.0% | 11 | 19.732356 | 13.459267 | +31.791% | 48.941010 | 33.696849 | 24.800230 | 19.762287 | -0.535% | 9 | 1 |
| 5.0% | 19 | 20.813255 | 20.165661 | +3.111% | 36.195719 | 36.075716 | 23.528471 | 25.295790 | -0.383% | 9 | 1 |
| 5.0% | 23 | 7.369449 | 6.476570 | +12.116% | 24.487945 | 13.266817 | 10.146024 | 8.998937 | -1.366% | 9 | 1 |

## Registered grid gate

| Margin | Mean gain | Positive seeds | Worst gain | Consensus gain | Max +byte | Max abs. byte | Hard | Track | Tail | Comm | Pass |
|--:|--:|--:|--:|--:|--:|--:|:--:|:--:|:--:|:--:|:--:|
| 0.0% | +24.862% | 1.000 | +30.234% | +18.646% | 0.251% | 0.465% | 1 | 1 | 1 | 1 | 1 |
| 2.5% | +18.718% | 1.000 | +32.702% | +12.778% | 0.000% | 2.189% | 1 | 1 | 1 | 0 | 0 |
| 5.0% | +16.307% | 1.000 | +24.251% | +7.555% | 0.000% | 1.366% | 1 | 1 | 1 | 1 | 1 |

- Mean-gain gate: `>= 5.000%`
- Maximum positive byte delta: `<= 0.500%`
- Maximum absolute byte delta: `<= 2.000%`
- Screen passed: `1`
- Selected margin: `0.0%` (token `000`)
- Next: `freeze-selected-margin-and-run-complete-six-seed-training-gate`

The payload-aware margin grid is selected only on opened M24 training states. The already-opened M24 development split may be used for redesign tuning but cannot be relabelled as validation. Held-out M24 and X36 remain sealed until one margin is frozen.
