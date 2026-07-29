# Backbone-preserving residual headroom audit

- Protocol: `backbone-preserving-additive-residual-routing-v2-e10`
- Preset: `m24-hard`
- Seed: `7`
- Source SHA-256: `3f957c6d41b91ddccdbeb28a55a4a9ee940e03aec549b216b24965d1b0ad1efe`
- Dominant/residual weights: `0.70 / 0.10`
- Baselines: `directed-fixed-index-w50, directed-fixed-index-w70, backbone-residual-static-a70-e10`
- Headroom gate passed: `0`
- Return-data generation authorized: `0`
- Evidence boundary: This development audit tests the additive residual action space against local-only, fixed-index w=0.50, fixed-index w=0.70, and an equal-payload static residual control. The candidate policies read truth and are nondeployable. A pass authorizes paired return-data generation only.

| Candidate | E-OSPA | Worst node | Min mean gain | Min tail gain | Local mean | Local tail | Byte dev. | B3 | Repair | Emergency | Pass |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `oracle-backbone-residual-current-a70-e10` | 18.4231 | 34.6283 | 2.07% | 0.00% | 21.47% | 17.64% | 0.20% | 1.000 | 0.000 | 0.000 | 0 |
| `oracle-backbone-residual-minimax-a70-e10` | 18.4231 | 34.6283 | 2.07% | 0.00% | 21.47% | 17.64% | 0.20% | 1.000 | 0.000 | 0.000 | 0 |
