# Backbone residual S/C policy-sequence audit

- Protocol: `backbone-preserving-residual-policy-sequence-m24-h3-v1`
- Gate passed: `0`
- Eligible sequences: `3/8`
- Evidence boundary: The best S/C mask is selected after observing all paired three-step outcomes and C steps read current truth. This is a nondeployable development diagnosis. A pass authorizes broader return-data generation only.

## Baselines

| Baseline | E-OSPA | Worst | Bytes |
|---|---:|---:|---:|
| `local` | 23.4600 | 42.0469 | 0 |
| `directed-fixed-index-w70` | 19.2470 | 34.6420 | 3512112 |
| `backbone-residual-static-a70-e05` | 18.8201 | 34.6311 | 5868888 |
| `backbone-residual-analytic-a70-e05` | 18.7966 | 38.6978 | 5860992 |

## Sequences

| Mask | E-OSPA | Worst | Min mean gain | Min tail gain | Byte dev. | B3 | Truth | Eligible | Pass |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `000` | 18.8201 | 34.6311 | -0.12% | 0.00% | 0.00% | 0 | 0.00 | 0 | 0 |
| `001` | 18.8201 | 34.6311 | -0.12% | 0.00% | 0.00% | 0 | 0.33 | 0 | 0 |
| `010` | 18.4530 | 34.6311 | 1.83% | 0.00% | 0.07% | 0 | 0.33 | 0 | 0 |
| `011` | 18.2128 | 34.6311 | 3.11% | 0.00% | 0.00% | 1 | 0.67 | 1 | 0 |
| `100` | 18.2827 | 34.6311 | 2.73% | 0.00% | 0.02% | 0 | 0.33 | 0 | 0 |
| `101` | 18.0414 | 34.6311 | 4.02% | 0.00% | 0.02% | 1 | 0.67 | 1 | 0 |
| `110` | 18.2847 | 34.6311 | 2.72% | 0.00% | 0.03% | 0 | 0.67 | 0 | 0 |
| `111` | 18.0445 | 34.6311 | 4.00% | 0.00% | 0.10% | 1 | 1.00 | 1 | 0 |

## Selected development upper bound

- Mask: `101`
- E-OSPA / worst: `18.0414 / 34.6311`
- Minimum baseline mean gain: `4.02%`
- Gate passed: `0`
