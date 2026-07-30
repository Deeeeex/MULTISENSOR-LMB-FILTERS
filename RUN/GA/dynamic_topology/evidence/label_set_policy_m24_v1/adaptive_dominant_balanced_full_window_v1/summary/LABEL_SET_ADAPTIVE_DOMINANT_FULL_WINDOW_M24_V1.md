# M24 adaptive-dominant balanced full-window gate

- Contract: `m24-adaptive-dominant-balanced-full-window-training-v1`
- Generated: 2026-07-31 01:37:55
- Run commit: `1f960446cc61722d363ba53ca0b77dae113d7415`
- Seeds: `[11 17 19 23 27 29]`
- Window: `[75 76 77 78 79 80 81 82 83]`
- Frozen arm: `adaptive-dominant-composite-balanced-e05-a70`
- Reference: `backbone-residual-spliced-cycle-ccw-a70-e05`

| Seed | CCW E-OSPA | D-B E-OSPA | Gain vs CCW | Legacy E-OSPA | Gain vs legacy | CCW worst | D-B worst | CCW consensus | D-B consensus | Byte delta | Msgs | Selected B3 | Delivered B3 | Safe |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| 11 | 19.732356 | 11.497345 | +41.734% | 17.585406 | +34.620% | 48.941010 | 28.594880 | 24.800230 | 16.994428 | +1.641% | 40.00 | 1.000 | 0.222 | 1 |
| 17 | 18.586537 | 13.151717 | +29.241% | 19.751387 | +33.414% | 34.658685 | 30.773268 | 21.100726 | 17.969190 | +0.542% | 40.00 | 1.000 | 0.556 | 1 |
| 19 | 20.813255 | 19.084506 | +8.306% | 16.774709 | -13.770% | 36.195719 | 37.459070 | 23.528471 | 23.550483 | +1.009% | 40.00 | 1.000 | 0.778 | 1 |
| 23 | 7.369449 | 5.336038 | +27.592% | 6.349615 | +15.963% | 24.487945 | 8.876269 | 10.146024 | 7.071841 | +0.763% | 40.00 | 1.000 | 0.889 | 1 |
| 27 | 15.497597 | 11.837624 | +23.616% | 17.361393 | +31.816% | 28.033309 | 31.395107 | 19.453617 | 16.812958 | -0.935% | 40.00 | 1.000 | 0.778 | 1 |
| 29 | 13.067441 | 5.685146 | +56.494% | 11.911126 | +52.270% | 30.821919 | 8.442354 | 16.884321 | 7.671548 | +0.344% | 40.00 | 1.000 | 0.556 | 1 |

## Registered gate

- Mean CCW E-OSPA: `15.844439`
- Mean D-B E-OSPA: `11.098729`
- Mean legacy E-OSPA: `14.955606`
- Mean gain vs CCW: `+29.952%` (gate `>= 5.000%`)
- Mean gain vs legacy: `+25.789%`
- Positive seeds: `6/6` (gate `>= 5/6`)
- Aggregate worst-node gain: `+28.354%`
- Aggregate consensus gain: `+22.295%`
- Maximum absolute attempted-byte delta: `1.641%` (gate `<= 2.000%`)
- Restart sentinel passed: `1`
- Hard safety passed: `1`
- Tracking gate passed: `1`
- Tail gate passed: `1`
- Communication gate passed: `1`
- Full training gate passed: `1`
- Next: `freeze-balanced-default-and-open-registered-M24-development`

This report evaluates a frozen truth-free controller only on the six opened M24 training seeds. It does not establish development, held-out M24, X36, or cross-scale generalization.
