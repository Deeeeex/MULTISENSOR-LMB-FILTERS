# Dynamic-topology oracle-gap screen

- Preset: `d12-handover`
- Seeds: `7`
- Generated: 2026-07-25 13:16:47
- Decision status: `screening-only`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Edges | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Local only | 69.5033 | 100.0000 | 0.4905 | 49.5405 | 2.0104 | 0 | 0 | 14.00 | 0.0000 | 0.0000 | 0.00 | 1.39 |
| Robust geometry static | 68.8854 | 100.0000 | 0.4720 | 57.1895 | 1.9688 | 1046416 | 1046416 | 14.00 | 0.0000 | 0.0000 | 0.00 | 4.25 |
| Reliability dynamic | 68.8854 | 100.0000 | 0.4720 | 57.1895 | 1.9688 | 1046416 | 1046416 | 14.00 | 0.0000 | 0.0000 | 0.04 | 4.42 |
| Posterior discrepancy dynamic | 68.0836 | 100.0000 | 0.4769 | 60.6769 | 1.9271 | 1047256 | 1047256 | 14.00 | 0.1333 | 0.0000 | 0.94 | 5.22 |
| Exact one-step consensus oracle | 67.8796 | 100.0000 | 0.4761 | 61.4368 | 1.9167 | 1037344 | 1037344 | 14.00 | 0.0202 | 0.0000 | 15.51 | 19.85 |
| Exact one-step truth diagnostic oracle | 66.8916 | 100.0000 | 0.4835 | 60.8691 | 1.8750 | 1036000 | 1036000 | 14.00 | 0.0000 | 0.0000 | 15.31 | 19.57 |

## Registered gate readout

- Oracle consensus gain: -0.87%
- Oracle tracking gain: 1.75%
- Analytic share of static-to-oracle gain: NaN
- Attempted-byte mismatch: 0.87%
- Recommendation: Treat this as a runtime and signal check; at least 10 paired screening seeds are required before a research stop decision.

## Evidence limits

- This runner isolates topology: every active edge sends the same heavy posterior every step; event triggering and payload compression are disabled.
- A one- or three-seed run is a software/runtime screen, not a paper-level effect estimate. The registered screening gate needs at least 10 paired seeds; the held-out claim needs 30.
- Equal edge budgets do not guarantee equal bytes. The table therefore reports attempted payload bytes explicitly.
