# Directed-routing held-out validation

- Generated: 2026-07-26 05:12:04
- Seeds: `17`
- Presets: `m24-hard, x36-clean-scale`
- Window: `[75 77]`
- Model: `RUN/GA/dynamic_topology/models/directed_routing_knn_m24_t75.mat`
- All mean-tracking gates pass: `0`
- All strict tail gates pass: `1`
- All topology checks feasible: `1`

| Preset | Seed | Local E-OSPA | Static E-OSPA | Learned E-OSPA | Gain vs static | Gain vs local | Worst gain vs static | Worst gain vs local | Bytes/static | Routes | Status |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--|
| m24-hard | 17 | 22.9856 | 22.9006 | 17.7871 | 22.33% | 22.62% | 21.02% | 9.96% | 26.75% | 16.67 | `directed-routing-screening-gain` |
| x36-clean-scale | 17 | 39.4500 | 43.8843 | 39.0351 | 11.05% | 1.05% | 6.15% | 0.00% | 15.55% | 16.00 | `directed-routing-below-mean-gate` |

## Evidence boundary

These seeds and time steps are excluded from model fitting. The runs remain conditional continuations from a common static prefix, not full-episode estimates. The model was selected on one M24 seed-7 t=75 dataset, and the KLA path remains the repository componentwise powered-GM approximation.
