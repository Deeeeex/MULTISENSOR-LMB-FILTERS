# X36 certified-overlap zero-shot scale validation

- Contract: `x36-clean-scale-adaptive-dominant-certified-overlap-zero-shot-v1`
- Generated: 2026-07-31 07:53:49
- Run commit: `55055e579ffaffa2cfe16a92146d430ac10b83ae`
- Preset: `x36-clean-scale`
- Margin: `2.0%`
- Seeds: `[41 43 47 53 59]`
- Window: `[75 76 77 78 79 80 81 82 83]`

| Seed | CCW E-OSPA | Candidate E-OSPA | Gain | Byte delta | Msgs | Maps | Safe |
|--:|--:|--:|--:|--:|--:|--:|:--:|
| 41 | 35.694015 | 35.886760 | -0.540% | -9.110% | 54.44 | 9 | 1 |
| 43 | 37.267553 | 34.358527 | +7.806% | -9.327% | 54.00 | 9 | 1 |
| 47 | 37.715377 | 29.391066 | +22.071% | -9.341% | 53.89 | 9 | 1 |
| 53 | 33.470572 | 30.333318 | +9.373% | -8.576% | 54.56 | 9 | 1 |
| 59 | 34.840180 | 34.876059 | -0.103% | -9.348% | 54.44 | 9 | 1 |

## Preregistered zero-shot scale gate

- Mean E-OSPA gain: `+7.901%`
- Positive-seed fraction: `0.600`
- Aggregate worst-node gain: `+8.535%`
- Aggregate consensus gain: `+0.322%`
- Aggregate attempted-byte saving: `+9.139%`
- Maximum positive byte delta: `+0.000%`
- Required minimum mean gain: `5.000%`
- Required positive-seed fraction: `0.800`
- Required minimum byte saving: `2.000%`
- Hard safety passed: `1`
- Tracking gate passed: `0`
- Tail gate passed: `1`
- Communication gate passed: `1`
- X36 scale gate passed: `0`
- Zero-shot X36 claim allowed: `0`
- Next: `stop-X36-scale-claim-without-retuning-on-these-results`

The method, 2-percent payload margin, CCW reference and gates were selected using M24 only. Older X36 seeds 7 and 17 are excluded. These five X36 clean-scale scenario-seed pairs were opened once from the frozen commit. The evidence is a paired conditional continuation over t=75:83, not a full episode.
