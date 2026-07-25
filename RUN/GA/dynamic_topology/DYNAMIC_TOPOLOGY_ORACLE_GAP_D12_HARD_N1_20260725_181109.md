# Dynamic-topology oracle-gap screen

- Preset: `d12-hard`
- Seeds: `7`
- Generated: 2026-07-25 18:11:09
- Decision status: `insufficient-arms`

- Focus window: `teacher-handover`, steps `[30 60]`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Edges | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| All-time geometry static | 78.3936 | 80.5076 | 0.4581 | 67.7921 | 6.2194 | 32026488 | 32026488 | 14.00 | 0.0000 | 0.0000 | 0.00 | 138.81 |
| Posterior discrepancy dynamic | 77.8018 | 81.6594 | 0.4506 | 67.8933 | 6.1500 | 32541840 | 32541840 | 14.00 | 0.1291 | 0.0000 | 58.63 | 198.07 |
| Pure current task-risk teacher | 69.6330 | 74.3052 | 0.4525 | 59.5673 | 4.8875 | 32127360 | 32127360 | 14.00 | 0.1142 | 0.0000 | 529.70 | 670.61 |

## Focus-window result

| Arm | Focus E-OSPA | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|
| All-time geometry static | 73.3108 | 0.4988 | 65.7286 | 6.5054 |
| Posterior discrepancy dynamic | 72.1001 | 0.4865 | 65.8284 | 6.3065 |
| Pure current task-risk teacher | 61.7041 | 0.4693 | 51.7472 | 4.5968 |

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
