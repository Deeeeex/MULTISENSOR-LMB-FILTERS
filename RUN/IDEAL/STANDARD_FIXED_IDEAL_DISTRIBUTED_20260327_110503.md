# Standard Fixed Ideal Distributed Compare (2026-03-27 11:05:03)

Ideal communication, 4-sensor ring graph, standard `Fixed` scenario.

## Run Config
- Trials: 5
- baseSeed: 1 (fixed=1)
- topology: ring
- neighborMap: [4 1 2;1 2 3;2 3 4;3 4 1]
- ideal pDropBySensor: [0 0 0 0]

## Consensus Metrics
| Mode | OSPA | RMSE | Card |
|:-----|-----:|-----:|-----:|
| Fixed distributed GA | 1.692102 | 2.363234 | 0.048500 |
| Current-best adaptive distributed GA | 1.701932 | 2.359601 | 0.050500 |

## Structure Priors
- Sensor 1 spatial prior: [0.9 1.2 0.9]
- Sensor 1 existence prior: [1.071 0.8571 1.071]
- Sensor 2 spatial prior: [0.9 1.2 0.9]
- Sensor 2 existence prior: [1.071 0.8571 1.071]
- Sensor 3 spatial prior: [0.9 1.2 0.9]
- Sensor 3 existence prior: [1.071 0.8571 1.071]
- Sensor 4 spatial prior: [0.9 1.2 0.9]
- Sensor 4 existence prior: [1.071 0.8571 1.071]
