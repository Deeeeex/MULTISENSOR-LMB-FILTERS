# Backbone-preserving residual headroom audit

- Protocol: `backbone-preserving-additive-residual-routing-v1`
- Preset: `m24-hard`
- Seed: `7`
- Source SHA-256: `b32abe778c0024578ade072bcbb5f700205079bdea8d58bd677c93245ef90f9d`
- Dominant/residual weights: `0.70 / 0.05`
- Baselines: `directed-fixed-index-w50, directed-fixed-index-w70, backbone-residual-static-a70-e05`
- Headroom gate passed: `0`
- Return-data generation authorized: `0`
- Evidence boundary: This development audit tests the additive residual action space against local-only, fixed-index w=0.50, fixed-index w=0.70, and an equal-payload static residual control. The candidate policies read truth and are nondeployable. A pass authorizes paired return-data generation only.

| Candidate | E-OSPA | Worst node | Min mean gain | Min tail gain | Local mean | Local tail | Byte dev. | B3 | Repair | Emergency | Pass |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `oracle-backbone-residual-current-a70-e05` | 18.0445 | 34.6311 | 4.12% | 0.00% | 23.08% | 17.64% | 0.10% | 1.000 | 0.000 | 0.000 | 0 |
| `oracle-backbone-residual-minimax-a70-e05` | 18.0445 | 34.6311 | 4.12% | 0.00% | 23.08% | 17.64% | 0.10% | 1.000 | 0.000 | 0.000 | 0 |
