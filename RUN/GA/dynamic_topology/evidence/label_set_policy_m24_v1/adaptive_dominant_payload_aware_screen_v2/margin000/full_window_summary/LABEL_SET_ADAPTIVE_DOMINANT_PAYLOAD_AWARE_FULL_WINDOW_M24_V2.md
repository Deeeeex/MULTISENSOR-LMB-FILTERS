# M24 payload-aware adaptive-dominant full-window gate

- Contract: `label-set-safe-adaptive-dominant-payload-budget-v2`
- Generated: 2026-07-31 02:55:16
- Run commits: `0a12e7641981e1dfd02954f8bf952ed87ac03988, 52562dda046c82112da6155ad211b950e1fed2d6`
- Selected margin: `0.0%`
- Seeds: `[11 17 19 23 27 29]`
- Window: `[75 76 77 78 79 80 81 82 83]`

| Seed | CCW E-OSPA | Candidate E-OSPA | Gain | CCW worst | Candidate worst | CCW consensus | Candidate consensus | Byte delta | Msgs | Selected B3 | Delivered B3 | Maps | Safe |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| 11 | 19.732356 | 12.011647 | +39.127% | 48.941010 | 28.469563 | 24.800230 | 17.623245 | -0.296% | 40.00 | 1.000 | 0.778 | 9 | 1 |
| 17 | 18.586537 | 16.123498 | +13.252% | 34.658685 | 32.079892 | 21.100726 | 20.155300 | +0.292% | 40.00 | 1.000 | 0.667 | 9 | 1 |
| 19 | 20.813255 | 17.879595 | +14.095% | 36.195719 | 33.204451 | 23.528471 | 21.520774 | +0.251% | 40.00 | 1.000 | 0.556 | 9 | 1 |
| 23 | 7.369449 | 6.111220 | +17.074% | 24.487945 | 14.806564 | 10.146024 | 8.427584 | -0.465% | 40.00 | 1.000 | 0.889 | 9 | 1 |
| 27 | 15.497597 | 13.130741 | +15.272% | 28.033309 | 33.334597 | 19.453617 | 18.136224 | -1.100% | 40.00 | 1.000 | 0.778 | 9 | 1 |
| 29 | 13.067441 | 5.792875 | +55.669% | 30.821919 | 8.728853 | 16.884321 | 7.793584 | -0.480% | 40.00 | 1.000 | 0.667 | 9 | 1 |

## Registered gate

- Mean CCW E-OSPA: `15.844439`
- Mean candidate E-OSPA: `11.841596`
- Mean gain: `+25.263%` (gate `>= 5.000%`)
- Positive seeds: `6/6`
- Aggregate worst-node gain: `+25.852%`
- Aggregate consensus gain: `+19.201%`
- Maximum positive byte delta: `0.292%`
- Maximum absolute byte delta: `1.100%`
- Hard safety passed: `1`
- Tracking gate passed: `1`
- Tail gate passed: `1`
- Communication gate passed: `1`
- Full-window gate passed: `1`
- Next: `run-opened-development-as-redesign-tuning-before-freezing-heldout`

The payload-aware margin grid is selected only on opened M24 training states. The already-opened M24 development split may be used for redesign tuning but cannot be relabelled as validation. Held-out M24 and X36 remain sealed until one margin is frozen.
