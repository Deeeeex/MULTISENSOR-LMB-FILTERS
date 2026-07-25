# Dynamic-topology oracle-gap screen

> Post-run audit: the `analytic share` value below is not interpretable because
> both one-step diagnostic references are dominated by the deployable
> discrepancy arm. The static arm is geometry-selected rather than an
> exhaustive fixed-graph performance optimum. Use
> `DYNAMIC_TOPOLOGY_FINDINGS_CN.md` for the corrected research classification.

- Preset: `d12-handover`
- Seeds: `[7 17 27]`
- Generated: 2026-07-25 15:39:39
- Decision status: `screening-only`

- Focus window: `handover`, steps `[35 95]`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Edges | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Robust geometry static | 69.9090 | 76.1495 | 0.4600 | 59.6910 | 4.0108 | 35888912 | 35888912 | 14.00 | 0.0000 | 0.0000 | 0.00 | 158.87 |
| Posterior discrepancy dynamic | 66.8892 | 74.2269 | 0.4270 | 57.1158 | 3.6564 | 36149720 | 36149720 | 14.00 | 0.1333 | 0.0000 | 63.83 | 222.15 |
| Exact one-step consensus oracle | 70.4648 | 77.7939 | 0.4573 | 60.0229 | 4.0708 | 35915272 | 35915272 | 14.00 | 0.0056 | 0.0000 | 661.79 | 821.62 |
| Exact one-step truth diagnostic oracle | 68.4871 | 75.1389 | 0.4593 | 58.6264 | 3.8447 | 35916096 | 35916096 | 14.00 | 0.0025 | 0.0000 | 664.73 | 822.21 |

## Focus-window result

| Arm | Focus E-OSPA | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|
| Robust geometry static | 67.1712 | 0.4468 | 54.6972 | 4.1102 |
| Posterior discrepancy dynamic | 62.6143 | 0.3984 | 50.1716 | 3.5788 |
| Exact one-step consensus oracle | 67.4696 | 0.4418 | 54.8073 | 4.1530 |
| Exact one-step truth diagnostic oracle | 65.5638 | 0.4466 | 53.5688 | 3.9185 |

## Registered gate readout

- Oracle consensus gain: -10.91%
- Oracle tracking gain: -4.71%
- Analytic share of static-to-oracle gain: 9.698
- Attempted-byte mismatch: 0.65%
- Recommendation: Treat this as a runtime and signal check; at least 10 paired screening seeds are required before a research stop decision.

## Evidence limits

- This runner isolates topology: every active edge sends the same heavy posterior every step; event triggering and payload compression are disabled.
- A one- or three-seed run is a software/runtime screen, not a paper-level effect estimate. The registered screening gate needs at least 10 paired seeds; the held-out claim needs 30.
- Equal edge budgets do not guarantee equal bytes. The table therefore reports attempted payload bytes explicitly.
