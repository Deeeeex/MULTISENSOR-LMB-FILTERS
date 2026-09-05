# V286: full-mixture output-risk source ranking

Final time 40, X36 seed 1301, 139 added-target snapshot queries. Diagnostic only / self-check only. No filter, routing action, packet cost or official metric changed.

For output a, risk = sum_c w_c [tr(P_c,position) + ||mu_c,position-a||^2]. This exact posterior expectation includes all GM modes; it is not a theorem about truth error or a new method. The retained self output and peer outputs are all from the same final snapshot; this is not an online decision.

Full objects match emitted label, timestamp at trajectoryLength (timestamps are preallocated), and a complete emitted mean/covariance pair. All 326 emitted objects are represented; 5 archived objects were excluded.

| Pool | Cases | Receiver | Truth oracle | Min-component trace | Min-full risk | Full risk worsens (%) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Global | 139 | 72.338283 | 28.936473 | 66.095875 | 65.223646 | 30.22 |
| Same formation | 139 | 72.338283 | 69.447822 | 70.006504 | 69.996701 | 29.50 |
| Planned incoming | 139 | 72.338283 | 71.195673 | 71.327333 | 71.327500 | 25.90 |
| Delivered incoming | 139 | 72.338283 | 71.224033 | 71.348804 | 71.348971 | 23.74 |
| Physical one-hop | 139 | 72.338283 | 63.122135 | 65.103081 | 65.103081 | 33.09 |

All displayed errors are pooled matched-position RMSE (m), not official average per-cell RMSE. Truth is used only for query construction, the explicitly privileged oracle, and evaluation; the two risk selectors never read truth. All selectors may retain self.

| Formation (global pool) | Cases | Receiver | Truth oracle | Min-component trace | Min-full risk | Full risk worsens (%) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 31 | 11.733638 | 8.421547 | 31.508679 | 31.508679 | 64.52 |
| 2 | 27 | 15.428177 | 7.990661 | 14.927643 | 14.927643 | 25.93 |
| 3 | 19 | 16.688747 | 3.336630 | 6.785402 | 6.000153 | 36.84 |
| 4 | 14 | 88.663363 | 21.496276 | 31.164323 | 26.535800 | 0.00 |
| 5 | 20 | 106.535293 | 38.820732 | 78.136830 | 76.931253 | 10.00 |
| 6 | 28 | 115.623210 | 51.976324 | 124.480645 | 123.291109 | 21.43 |

## Descriptive risk/error check

Source-truth pairs are collapsed within the snapshot to avoid counting the same sender state repeatedly for multiple receiver queries. They still share labels, measurements and previous fusion, so correlations are descriptive, not independent calibration evidence. The entire GM spatial distribution is conditional on existence; this score does not evaluate missed/false targets or recursive set effects.

| Source formation | Pairs | Actual MSE | Component trace | Full risk | Log-risk/error correlation: component / full | Multi-GM (%) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 273 | 3362.741 | 83056.291 | 83056.487 | 0.334 / 0.334 | 6.2 |
| 1 | 68 | 262.889 | 109571.221 | 109571.841 | 0.165 / 0.164 | 11.8 |
| 2 | 60 | 315.855 | 95547.564 | 95547.474 | 0.207 / 0.206 | 10.0 |
| 3 | 26 | 852.267 | 137082.461 | 137082.461 | -0.776 / -0.776 | 0.0 |
| 4 | 18 | 7314.738 | 24932.129 | 24932.129 | 0.178 / 0.178 | 0.0 |
| 5 | 48 | 7610.323 | 44964.195 | 44964.549 | 0.876 / 0.876 | 6.2 |
| 6 | 53 | 6831.714 | 62631.448 | 62631.448 | 0.902 / 0.902 | 0.0 |

Maximum algebraic risk-decomposition residual 6.13e-13. No past full-mixture snapshots were inferred from trajectory means; no aged or full-episode claim is supported. A favorable snapshot score would still require actual causal transport, charged metadata, losses, and static-routing comparison under identical fusion.
