# Backbone residual adaptive-trust audit

- Protocol: `backbone-residual-spliced-cycle-adaptive-trust-m24-h3-v1`
- Gate passed: `0`
- Constraint eligible: `1`
- Tail-safe refinement required: `1`
- Evidence boundary: The adaptive edge-weight teacher reads current truth and is a nondeployable development diagnostic. Deterministic paired baselines are reused only after an exact SHA-256 match. A pass would authorize return-data generation only.

## Results

| Arm | E-OSPA | Worst |
|---|---:|---:|
| `local` | 23.4600 | 42.0469 |
| `directed-fixed-index-w70` | 19.2470 | 34.6420 |
| `backbone-residual-static-a70-e05` | 18.8201 | 34.6311 |
| `backbone-residual-analytic-a70-e05` | 18.7966 | 38.6978 |
| `backbone-residual-spliced-cycle-cw-a70-e05` | 19.6230 | 34.6518 |
| `backbone-residual-spliced-cycle-ccw-a70-e05` | 18.9961 | 34.6300 |
| `oracle-backbone-residual-spliced-cycle-adaptive-current-a70` | 17.7051 | 34.6340 |

- Minimum baseline mean gain: `5.8066%`
- Minimum baseline worst-node gain: `-0.0116%`
- Attempted-byte deviation: `0.0204%`
- Mean cross residual weight / temporal spread: `0.0833 / 0.1000`
- Independent sensor / formation B3: `[true true true] / [true true true]`
- Mean / tail / bytes / safety / provenance / cross gates: `1 / 0 / 1 / 1 / 1 / 1`
