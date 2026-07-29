# Backbone residual CVaR-tail-guard audit

- Protocol: `backbone-residual-cvar-tail-guard-adaptive-trust-m24-h3-v1`
- Gate passed: `1`
- Constraint eligible: `1`
- Tail-safe refinement required: `0`
- Evidence boundary: The edge-and-trust objective, current E-OSPA active set, and hard reference guards read current truth and form a nondeployable development headroom diagnostic. Deterministic paired baselines are reused only after an exact SHA-256 match. A pass authorizes return-data generation, not a deployable or held-out claim.

## Results

| Arm | E-OSPA | Worst |
|---|---:|---:|
| `local` | 23.4600 | 42.0469 |
| `directed-fixed-index-w70` | 19.2470 | 34.6420 |
| `backbone-residual-static-a70-e05` | 18.8201 | 34.6311 |
| `backbone-residual-analytic-a70-e05` | 18.7966 | 38.6978 |
| `backbone-residual-spliced-cycle-cw-a70-e05` | 19.6230 | 34.6518 |
| `backbone-residual-spliced-cycle-ccw-a70-e05` | 18.9961 | 34.6300 |
| `oracle-backbone-residual-spliced-cycle-cvar-tail-guard-adaptive-current-a70` | 17.7119 | 34.6286 |

- Minimum baseline mean gain: `5.7707%`
- Minimum baseline worst-node gain: `0.0039%`
- Attempted-byte deviation: `0.0299%`
- Mean cross residual weight / temporal spread: `0.0833 / 0.1000`
- Independent sensor / formation B3: `[true true true] / [true true true]`
- Mean / tail / bytes / safety / provenance / cross gates: `1 / 1 / 1 / 1 / 1 / 1`
