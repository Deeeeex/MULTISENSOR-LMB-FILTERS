# GA Posterior Structure Comparison (2026-04-08 00:29:01)

Comparison order: static weak structure prior -> posterior-structure-consistency

## Run Config
- Trials: 5
- baseSeed: 1 (fixed=1)

## Arm Configs
### Static weak structure prior
- usePosteriorStructureConsistency: 0

### Posterior-structure-consistency
- usePosteriorStructureConsistency: 1

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| static weak structure prior | 1.862244 | 1.749608 | 0.244250 |
| posterior-structure-consistency | 1.862244 | 1.749608 | 0.244250 |

## Aggregated Local Metrics (mean across sensors)
- E-OSPA: 2.381696 -> 2.381696
- RMSE: 1.602228 -> 1.602228
