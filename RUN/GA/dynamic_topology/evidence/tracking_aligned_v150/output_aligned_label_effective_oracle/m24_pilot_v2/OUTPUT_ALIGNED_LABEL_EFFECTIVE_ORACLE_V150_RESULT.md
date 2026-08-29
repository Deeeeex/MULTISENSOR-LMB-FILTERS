# V150 output-aligned label-effective oracle

- Case: `m24-formation-fov / seed 211 / t=104 / H=8`
- Candidate bank: `12 bounded singleton omissions`
- Positive singleton count: `1`
- Bundle sizes evaluated: `[2 4 8]`
- Best source / action: `bundle / current-fused-impact-top-8`
- Development-only: `true`

## Best realized result

| Metric | Gain | Gate |
|:--|--:|:--:|
| Mean E-OSPA | +1.038% | FAIL |
| Worst sensor E-OSPA | +1.265% | PASS |
| Minimum formation E-OSPA | -0.000% | FAIL |
| Window consensus | +1.825% | PASS |
| Terminal consensus | +4.518% | PASS |
| Attempted bytes saved | -0.749% | FAIL |

Overall registered gate: `FAIL`

## Singleton outcomes

| Action | Mean gain | Worst gain | Min formation | Bytes saved |
|:--|--:|--:|--:|--:|
| `reference-full-payload` | +0.000% | +0.000% | +0.000% | +0.000% |
| `omit-r2-s9-l1-4` | +0.059% | +0.000% | -0.000% | -0.227% |
| `omit-r8-s15-l9-7` | +0.066% | +0.616% | -0.000% | -0.141% |
| `omit-r14-s21-l25-15` | +0.000% | +0.000% | +0.000% | +0.029% |
| `omit-r2-s9-l17-9` | +0.059% | +0.000% | +0.000% | -0.087% |
| `omit-r14-s21-l9-7` | +0.000% | +0.000% | +0.000% | +0.029% |
| `omit-r2-s9-l25-13` | +0.709% | +0.000% | +0.000% | -0.116% |
| `omit-r8-s15-l9-6` | +0.060% | +0.614% | +0.000% | -0.128% |
| `omit-r2-s9-l17-11` | +0.055% | +0.000% | -0.001% | -0.116% |
| `omit-r8-s15-l25-14` | -0.057% | +0.613% | -0.192% | -0.286% |
| `omit-r20-s3-l17-9` | +0.071% | +0.000% | +0.000% | -0.108% |
| `omit-r14-s21-l9-6` | +0.000% | +0.000% | +0.000% | +0.024% |
| `omit-r2-s9-l17-10` | +0.724% | +0.000% | +0.000% | +0.032% |

## Outcome-ranked bundle outcomes

| Action | Mean gain | Worst gain | Min formation | Bytes saved |
|:--|--:|--:|--:|--:|
| `reference-full-payload` | +0.000% | +0.000% | +0.000% | +0.000% |
| `current-fused-impact-top-2` | +0.125% | +0.616% | -0.000% | -0.368% |
| `current-fused-impact-top-4` | +0.188% | +0.616% | -0.000% | -0.425% |
| `current-fused-impact-top-8` | +1.038% | +1.265% | -0.000% | -0.749% |

## Evidence boundary

V150 is a privileged two-scale development oracle. Candidate label omissions are bounded and ranked from the current opened LMB posterior only. Ordinary single-state mixture-aware LMB-KLA replay scores recursive H=8 tracking outcomes. Future outcomes may rank and compose actions only inside this oracle and cannot support a deployable-policy, training, validation, or generalization claim.
