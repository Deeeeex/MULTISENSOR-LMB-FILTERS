# GA Association Ambiguity Comparison (2026-04-08 00:39:30)

Comparison order: three-factor baseline -> +association ambiguity

## Run Config
- Trials: 5
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

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| three-factor baseline | 1.874840 | 1.779820 | 0.244500 |
| +association ambiguity | 1.876368 | 1.769102 | 0.245500 |

## Aggregated Local Metrics (mean across sensors)
- E-OSPA: 2.382275 -> 2.382909
- RMSE: 1.599144 -> 1.599274
