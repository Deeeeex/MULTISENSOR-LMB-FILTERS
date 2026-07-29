# Backbone residual spliced-cycle audit

- Protocol: `backbone-residual-spliced-strong-cycle-m24-h3-v1`
- Gate passed: `0`
- Constraint eligible: `1`
- Evidence boundary: The current-risk cycle reads current truth and is a nondeployable development diagnostic. The executed selected topology is independently rechecked against the frozen B=3 history. A pass would authorize return-data generation only.

## Results

| Arm | E-OSPA | Worst |
|---|---:|---:|
| `local` | 23.4600 | 42.0469 |
| `directed-fixed-index-w70` | 19.2470 | 34.6420 |
| `backbone-residual-static-a70-e05` | 18.8201 | 34.6311 |
| `backbone-residual-analytic-a70-e05` | 18.7966 | 38.6978 |
| `backbone-residual-spliced-cycle-cw-a70-e05` | 19.6230 | 34.6518 |
| `backbone-residual-spliced-cycle-ccw-a70-e05` | 18.9961 | 34.6300 |
| `oracle-backbone-residual-spliced-cycle-current-a70-e05` | 18.0567 | 34.6314 |

- Minimum baseline mean gain: `3.9363%`
- Minimum baseline worst-node gain: `-0.0041%`
- Attempted-byte deviation: `0.1125%`
- Independent sensor B3: `[true true true]`
- Independent formation B3: `[true true true]`
- Repair / emergency / infeasible: `0 / 0 / 0`
- Mean cross-formation edges: `4.0000` (expected `4`)
- Mean / tail / bytes / safety / provenance / cross gates: `0 / 0 / 1 / 1 / 1 / 1`
