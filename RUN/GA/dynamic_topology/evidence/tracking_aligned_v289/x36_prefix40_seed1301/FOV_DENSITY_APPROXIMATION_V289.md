# V289: cached FoV density-integration diagnostic

Scope: x36-formation-fov-temporal-coupled-formation-braid, seed 1301, steps 1--40; 33786 predicted label stages. No filter rerun, target truth, new routing or tracking-performance result.

The runtime uses sum_j w_j pD(mu_j). This diagnostic estimates sum_j w_j E[pD(X_j)] with the same 120-degree, 300-m sensing model and state-dependent quality.

| Quantity | Result |
| --- | ---: |
| Mean absolute pD difference | 0.02230210 |
| 95th percentile absolute pD difference | 0.12777307 |
| Maximum absolute pD difference | 0.46564017 |
| Label stages with absolute pD difference >= 0.05 | 4147 / 33786 |
| Mean-pD-zero stages with density pD >= 0.05 | 3189 / 27939 |
| r >= 0.5 stages with absolute pD difference >= 0.05 | 1040 / 5950 |
| Existence-weighted mean absolute pD difference | 0.02787860 |
| Hypothetical no-detection mean absolute existence difference | 0.00185495 |
| Hypothetical no-detection maximum absolute existence difference | 0.16860724 |
| Hypothetical no-detection mean position difference, m | 3.56733243 |
| Hypothetical no-detection 95th percentile position difference, m | 16.89422140 |
| 2048 vs 8192 point mean / maximum absolute pD difference | 0.00031346 / 0.00423114 |
| Top twelve cases: 8192 vs 65536 point maximum pD difference | 0.00050703 |
| Runtime vs vectorized point-pD maximum difference | 3.33e-16 |

The no-detection values are isolated likelihood counterfactuals, not the actual posterior at every stage: the cached update may include detections and association competition. They evaluate r*(1-pDbar)/(1-r*pDbar) and the normalized density (1-pD(x))*p(x). Replacing only scalar pD does not reproduce this spatial update.

Numerical method: fixed unrandomized 2D Halton normal quadrature (2048 and 8192 points), refined to 65536 on the twelve largest absolute differences. Sensitivity values are numerical diagnostics, not confidence intervals or rigorous integration-error bounds. Component covariance is marginalized to position; all original mixture weights are retained.

Source trace: `RUN/GA/dynamic_topology/evidence/tracking_aligned_v282/x36_prefix40_seed1301/EXISTENCE_STAGE_TRACE_V282.mat`, source commit `fbf17cdd98439080b0befb2b5246f459aa8d074e`. Runtime 103.2 seconds. Self-check only; single opened development episode, not independent validation.
