# Dynamic-topology oracle-gap screen

- Preset: `d12-hard`
- Seeds: `7`
- Generated: 2026-07-25 18:14:26
- Decision status: `insufficient-arms`

- Focus window: `teacher-handover`, steps `[30 60]`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Edges | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Registered fixed candidate 16 | 73.8823 | 77.4447 | 0.4685 | 63.8313 | 5.4972 | 31722816 | 31722816 | 14.00 | 0.0000 | 0.0000 | 0.00 | 136.52 |

## Focus-window result

| Arm | Focus E-OSPA | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|
| Registered fixed candidate 16 | 67.5372 | 0.5100 | 58.4958 | 5.4946 |

## Registered gate readout

- Oracle consensus gain: NaN%
- Oracle tracking gain: NaN%
- Analytic share of static-to-oracle gain: NaN
- Diagnostic reference dominated: `0`
- Attempted-byte mismatch: NaN%
- Recommendation: Run the complete registered arm set.

## Evidence limits

- This runner isolates topology: every active edge sends the same heavy posterior every step; event triggering and payload compression are disabled.
- A one- or three-seed run is a software/runtime screen, not a paper-level effect estimate. The registered screening gate needs at least 10 paired seeds; the held-out claim needs 30.
- Equal edge budgets do not guarantee equal bytes. The table therefore reports attempted payload bytes explicitly.
- The static arm is selected by all-time geometry and link distance; it is not an exhaustive offline performance optimum over all fixed D12 candidates.
- Exact one-step action enumeration is not a closed-loop upper bound. If it is dominated by a deployable arm, it cannot justify a learned teacher or an analytic-sufficiency claim.
