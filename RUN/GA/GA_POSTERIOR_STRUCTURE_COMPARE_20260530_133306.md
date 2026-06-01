# GA Posterior Structure Comparison (2026-05-30 13:33:06)

Comparison order: static weak structure prior -> posterior-structure-consistency

## Run Config
- Trials: 50
- baseSeed: 1 (fixed=1)

## Arm Configs
### Static weak structure prior
- usePosteriorStructureConsistency: 0

### Posterior-structure-consistency
- usePosteriorStructureConsistency: 1

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| static weak structure prior | 1.791239 | 1.534119 | 0.187875 |
| posterior-structure-consistency | 1.791239 | 1.534119 | 0.187875 |

## Aggregated Local Metrics (mean across sensors)
- E-OSPA: 2.332596 -> 2.332596
- RMSE: 1.601376 -> 1.601376
