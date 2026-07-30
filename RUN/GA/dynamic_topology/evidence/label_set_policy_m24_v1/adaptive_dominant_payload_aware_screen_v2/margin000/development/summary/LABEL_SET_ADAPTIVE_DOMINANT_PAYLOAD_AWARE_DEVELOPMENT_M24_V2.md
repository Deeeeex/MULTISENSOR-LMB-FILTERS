# M24 payload-aware adaptive-dominant development tuning

- Contract: `label-set-safe-adaptive-dominant-payload-budget-v2`
- Generated: 2026-07-31 03:08:20
- Run commit: `d21c7a77c7b267f71660ea5669ac1f3b2b73a326`
- Selected margin: `0.0%`
- Seeds: `[31 37]`
- Window: `[75 76 77 78 79 80 81 82 83]`

| Seed | CCW E-OSPA | Candidate E-OSPA | Gain | CCW worst | Candidate worst | CCW consensus | Candidate consensus | Byte delta | Msgs | Selected B3 | Delivered B3 | Maps | Safe |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| 31 | 9.358520 | 7.318153 | +21.802% | 16.727171 | 20.868685 | 12.346338 | 10.609909 | +1.307% | 40.00 | 1.000 | 0.778 | 9 | 1 |
| 37 | 7.746187 | 5.491666 | +29.105% | 17.598772 | 10.584862 | 10.310441 | 7.471513 | -0.371% | 40.00 | 1.000 | 1.000 | 9 | 1 |

## Redesign tuning gate

- Mean CCW E-OSPA: `8.552354`
- Mean candidate E-OSPA: `6.404910`
- Mean gain: `+25.109%`
- Positive seeds: `2/2`
- Aggregate worst-node gain: `+8.368%`
- Aggregate consensus gain: `+20.194%`
- Maximum positive byte delta: `1.307%`
- Maximum absolute byte delta: `1.307%`
- Hard safety passed: `1`
- Tracking gate passed: `1`
- Tail gate passed: `1`
- Communication gate passed: `0`
- Development tuning gate passed: `0`
- Fresh validation claim allowed: `0`
- Next: `stop-without-opening-heldout-M24-or-X36`

Seeds 31 and 37 were already opened by D-B v1. These results may tune and freeze v2 but are not fresh validation evidence. Held-out M24 and X36 remain sealed in this report.
