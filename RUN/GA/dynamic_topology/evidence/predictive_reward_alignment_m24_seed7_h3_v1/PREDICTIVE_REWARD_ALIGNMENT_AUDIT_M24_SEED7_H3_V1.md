# M24 predictive reward alignment audit

- Generated: 2026-07-30 03:33:10
- Protocol: `predictive-reward-alignment-m24-seed7-h3-v1`
- Evidence commit: `adc35c4c850c2baeb2a197a75b1728a274379da8`
- Action times: `[75 76]`; delayed reward times: `[76 77]`
- Score: `poisson-measurement-intensity-log-score`; truth used by score: `0`
- Evidence boundary: The predictive score reads measurements but no target truth. The headroom arm itself reads current truth, so this training-only audit tests reward alignment and cannot establish deployable policy performance or authorize development/held-out evaluation.

| Arm | Predictive score | Focus E-OSPA | Worst node | Attempted bytes |
|:--|--:|--:|--:|--:|
| `backbone-residual-static-a70-e05` | -160.367438 | 18.8201 | 34.6311 | 5868888 |
| `backbone-residual-analytic-a70-e05` | -160.390917 | 18.7966 | 38.6978 | 5860992 |
| `oracle-backbone-residual-spliced-cycle-cvar-tail-guard-adaptive-current-a70` | -160.386361 | 17.7119 | 34.6286 | 5867136 |

| Reference | Mean score advantage [95% CI] | Paired Spearman [95% CI] | Affected pairs | Direction agreement | Positive tracking gain |
|:--|--:|--:|--:|--:|--:|
| `backbone-residual-static-a70-e05` | -0.064877 [-0.222931, 0.061371] | -0.3714 [-0.9006, 0.2328] | 14 | 0.286 | 0.786 |
| `backbone-residual-analytic-a70-e05` | 0.011510 [-0.223091, 0.344937] | 0.0211 [-0.5716, 0.6259] | 19 | 0.526 | 0.632 |

- Arm-rank Spearman: `-0.5000`
- Teacher score advantage over best reference: `-0.018922`
- Minimum paired sensor-time Spearman: `-0.3714`
- Minimum affected pair count: `14`
- Maximum attempted-byte deviation: `0.1048%`
- Gates (provenance / paired measurements / score advantage / arm rank / paired sensor-time / affected count / bytes / safety): `1 / 1 / 0 / 0 / 0 / 1 / 1 / 1`
- Alignment gate passed: `0`
- Bandit implementation authorized: `0`
- Development / held-out M24 / X36 authorized: `0 / 0 / 0`
