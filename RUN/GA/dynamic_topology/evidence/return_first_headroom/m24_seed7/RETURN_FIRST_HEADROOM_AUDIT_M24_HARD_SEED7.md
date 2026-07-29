# Return-first dynamic-graph headroom audit

- Protocol: `return-first-rolling-safe-graph-value-v1`
- Preset: `m24-hard`
- Seed: `7`
- Source SHA-256: `e2f8eb3fbba47c4741c444e3e57627feea38d1ba524205a8c73a85ec4fc32232`
- Safety backbone: `fixed-balanced-cycle`
- Baselines: `directed-fixed-index-w50, directed-fixed-cycle-w50`
- Headroom gate passed: `0`
- Return-data generation authorized: `0`
- Critic training authorized: `0`
- Evidence boundary: This development audit tests whether the exact rolling-B3 action space has tracking headroom against local-only and both same-weight static directed controls. The candidate policies read truth and are nondeployable. A pass authorizes paired H=3 return-data generation only; it does not authorize critic, held-out, X36-transfer or paper-level claims.

| Candidate | E-OSPA | Worst node | Min mean gain | Min tail gain | Local mean | Local tail | Byte dev. | B3 | Repair | Emergency | Pass |
|:--|--:|--:|--:|--:|--:|--:|--:|:--:|--:|--:|:--:|
| `oracle-rolling-safe-current-w50` | 21.6318 | 35.0717 | -8.06% | 0.20% | 7.79% | 16.59% | 0.80% | 1 | 0.0000 | 0.0000 | 0 |
| `oracle-rolling-safe-minimax-w50` | 21.4258 | 35.0938 | -7.03% | 0.14% | 8.67% | 16.54% | 1.01% | 1 | 0.0000 | 0.0000 | 0 |
