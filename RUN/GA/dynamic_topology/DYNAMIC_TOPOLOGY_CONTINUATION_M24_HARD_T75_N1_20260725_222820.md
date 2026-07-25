# Dynamic-topology oracle-gap screen

- Preset: `m24-hard`
- Seeds: `7`
- Generated: 2026-07-25 22:28:20
- Decision status: `best-observed-screening`

- Focus window: `teacher-handover-and-blockage`, steps `[75 80]`

- Analysis window: steps `[75 80]`
- Conditional continuation: all arms share the same static-prefix local posterior at `t=75`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Edges | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| All-time geometry static | 23.0892 | 47.3908 | 0.8205 | 25.6338 | 0.8056 | 17091984 | 16205472 | 29.00 | 0.0000 | 0.0000 | 0.00 | 70.43 |
| Pure current task-risk teacher | 21.1925 | 35.9637 | 0.8446 | 23.8603 | 0.6944 | 17101848 | 15619968 | 29.00 | 0.0227 | 0.0000 | 135.15 | 205.92 |

## Focus-window result

| Arm | Focus E-OSPA | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|
| All-time geometry static | 23.0892 | 0.8205 | 25.6338 | 0.8056 |
| Pure current task-risk teacher | 21.1925 | 0.8446 | 23.8603 | 0.6944 |

## Registered gate readout

- Constraint-eligible arms: `2`
- Best observed arm: `Pure current task-risk teacher`
- Best observed focus E-OSPA: 21.1925
- Best observed gain vs static: 8.21%
- Best observed byte mismatch vs static: 0.06%
- Oracle consensus gain: NaN%
- Oracle tracking gain: NaN%
- Analytic share of static-to-oracle gain: NaN
- Diagnostic reference dominated: `0`
- Attempted-byte mismatch: NaN%
- Recommendation: A communication-matched best observed strategy exists, but this remains a bounded screening result.

## Evidence limits

- This runner isolates topology: every active edge sends the same heavy posterior every step; event triggering and payload compression are disabled.
- A one- or three-seed run is a software/runtime screen, not a paper-level effect estimate. The registered screening gate needs at least 10 paired seeds; the held-out claim needs 30.
- Equal edge budgets do not guarantee equal bytes. The table therefore reports attempted payload bytes explicitly.
- This is a conditional continuation experiment. It compares policy decisions after a common static prefix and does not estimate full-episode performance.
- The static arm is selected by all-time geometry and link distance; it is not an exhaustive offline performance optimum.
- Exact one-step action enumeration is not a closed-loop upper bound. If it is dominated by a deployable arm, it cannot justify a learned teacher or an analytic-sufficiency claim.
