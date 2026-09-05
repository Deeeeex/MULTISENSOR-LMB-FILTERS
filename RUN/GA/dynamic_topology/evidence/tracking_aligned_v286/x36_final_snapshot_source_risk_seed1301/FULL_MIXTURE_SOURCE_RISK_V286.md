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

Source-label-truth states are collapsed within the snapshot to avoid repeating the same source state for receiver queries. Distinct labels assigned the same truth are retained. Shared labels, observations and prior fusion make correlations descriptive, not independent calibration evidence. Same symbolic labels do not prove the same target identity. The conditional spatial risk does not evaluate missed/false targets or recursive set effects.

| Source formation | Source-label-truth states | Actual MSE | Component trace | Full risk | Log-risk/error correlation: component / full | Multi-GM (%) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 503 | 6800.476 | 78326.345 | 78326.563 | 0.231 / 0.232 | 8.2 |
| 1 | 136 | 5810.120 | 78846.826 | 78847.440 | -0.232 / -0.233 | 9.6 |
| 2 | 126 | 6350.626 | 67645.211 | 67645.180 | -0.281 / -0.279 | 19.0 |
| 3 | 28 | 798.380 | 141036.154 | 141036.154 | -0.774 / -0.774 | 0.0 |
| 4 | 34 | 7844.131 | 37624.569 | 37624.569 | 0.430 / 0.430 | 0.0 |
| 5 | 76 | 8722.497 | 59116.657 | 59117.050 | 0.888 / 0.889 | 5.3 |
| 6 | 103 | 8527.371 | 101267.701 | 101267.701 | 0.936 / 0.936 | 0.0 |

Maximum algebraic risk-decomposition residual 6.13e-13. No past full-mixture snapshots were inferred from trajectory means; no aged or full-episode claim is supported. A favorable snapshot score would still require actual causal transport, charged metadata, losses, and static-routing comparison under identical fusion.
