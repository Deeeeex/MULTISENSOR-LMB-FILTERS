# GA Association Ambiguity Comparison (2026-05-30 04:29:25)

Comparison order: three-factor baseline -> +association ambiguity

## Run Config
- Trials: 50
- baseSeed: 1 (fixed=1)

## Arm Configs
### Three-factor baseline
- useAssociationAmbiguity: 0
- associationAmbiguityMinScore: 0.850
- associationAmbiguityPower: 1.000

### +association ambiguity
- useAssociationAmbiguity: 1
- associationAmbiguityMinScore: 0.850
- associationAmbiguityPower: 1.000

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| three-factor baseline | 1.789144 | 1.528563 | 0.187550 |
| +association ambiguity | 1.789175 | 1.528330 | 0.187675 |

## Aggregated Local Metrics (mean across sensors)
- E-OSPA: 2.330906 -> 2.331178
- RMSE: 1.601082 -> 1.601028
