# Dynamic-topology oracle-gap screen

- Preset: `x36-clean-scale`
- Seeds: `7`
- Generated: 2026-07-26 00:52:30
- Decision status: `stop-negligible-observed-gain`

- Focus window: `clean-scale-handover-and-blockage`, steps `[75 75]`

- Analysis window: steps `[75 75]`
- Conditional continuation: all arms share the same static-prefix local posterior at `t=75`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Edges | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| All-time geometry static | 46.8048 | 68.6548 | 0.9986 | 36.8924 | 2.4444 | 6648128 | 6494320 | 44.00 | 0.0000 | 0.0000 | 0.00 | 30.47 |
| Reliability dynamic | 46.3778 | 68.6548 | 0.9987 | 35.7304 | 2.3889 | 6654152 | 6500344 | 44.00 | 0.0444 | 0.0000 | 10.93 | 42.01 |
| Posterior discrepancy dynamic | 48.3960 | 68.6511 | 0.9724 | 36.0238 | 2.5556 | 6647888 | 6494080 | 44.00 | 0.1277 | 0.0000 | 35.07 | 65.94 |
| Pure current task-risk teacher | 46.3788 | 68.6548 | 0.9983 | 35.6645 | 2.3889 | 6660416 | 6429368 | 44.00 | 0.0444 | 0.0000 | 46.63 | 78.35 |
| Balanced current task-risk teacher | 46.3778 | 68.6548 | 0.9987 | 35.7304 | 2.3889 | 6654152 | 6500344 | 44.00 | 0.0444 | 0.0000 | 46.68 | 77.64 |
| Tail-risk current task-risk teacher | 46.3778 | 68.6548 | 0.9987 | 35.7304 | 2.3889 | 6654152 | 6500344 | 44.00 | 0.0444 | 0.0000 | 46.62 | 77.64 |

## Focus-window result

| Arm | Focus E-OSPA | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|
| All-time geometry static | 46.8048 | 0.9986 | 36.8924 | 2.4444 |
| Reliability dynamic | 46.3778 | 0.9987 | 35.7304 | 2.3889 |
| Posterior discrepancy dynamic | 48.3960 | 0.9724 | 36.0238 | 2.5556 |
| Pure current task-risk teacher | 46.3788 | 0.9983 | 35.6645 | 2.3889 |
| Balanced current task-risk teacher | 46.3778 | 0.9987 | 35.7304 | 2.3889 |
| Tail-risk current task-risk teacher | 46.3778 | 0.9987 | 35.7304 | 2.3889 |

## Registered gate readout

- Constraint-eligible arms: `6`
- Best observed arm: `Reliability dynamic`
- Best observed focus E-OSPA: 46.3778
- Best observed gain vs static: 0.91%
- Minimum practical tracking gain: 5.00%
- Best observed byte mismatch vs static: 0.09%
- Oracle consensus gain: NaN%
- Oracle tracking gain: NaN%
- Analytic share of static-to-oracle gain: NaN
- Diagnostic reference dominated: `0`
- Attempted-byte mismatch: NaN%
- Recommendation: The best observed communication-matched gain is below the registered 5.00% practical-effect threshold.

## Evidence limits

- This runner isolates topology: every active edge sends the same heavy posterior every step; event triggering and payload compression are disabled.
- A one- or three-seed run is a software/runtime screen, not a paper-level effect estimate. The registered screening gate needs at least 10 paired seeds; the held-out claim needs 30.
- Equal edge budgets do not guarantee equal bytes. The table therefore reports attempted payload bytes explicitly.
- This is a conditional continuation experiment. It compares policy decisions after a common static prefix and does not estimate full-episode performance.
- The static arm is selected by all-time geometry and link distance; it is not an exhaustive offline performance optimum.
- Exact one-step action enumeration is not a closed-loop upper bound. If it is dominated by a deployable arm, it cannot justify a learned teacher or an analytic-sufficiency claim.
