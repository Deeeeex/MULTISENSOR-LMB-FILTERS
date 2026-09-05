# V285: same-label spatial information in saved X36 outputs

Output-level diagnostic / optimistic bound, self-check only. No filter run, new message, policy evaluation or joint-gate revision. All 7850 added geometrically matched target cases are retained.

Lag 0 uses current end-of-round outputs and cannot be inserted into the same round before they exist. Lag 1 predicts the previous round with the registered 4D one-step CV model; it does not include the next measurement or simulate packet fusion. Pools use sender-to-receiver attempted/delivered masks.

The physical one-hop pool is reconstructed from the saved sensor trajectories with the registered 270 m range rule, not from selected edges. Every attempted edge lies inside it. Unselected physical edges have not been charged or subjected to a new packet-delivery trial.

Oracle selects by truth error and may retain the receiver. Min-trace selects only by the emitted component covariance trace (ties retain receiver); source selection excludes truth, but the query set, receiver snapshot timing and uncharged source synopsis make this a diagnostic, not a deployable controller.

| Lag | Source pool | Cases | Peer available (%) | Receiver pooled RMSE | Oracle pooled RMSE | Min-trace pooled RMSE | Min-trace worsens (%) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | All network peers | 7850 | 99.99 | 34.730995 | 11.104842 | 27.303868 | 46.64 |
| 0 | Same formation | 7850 | 96.68 | 34.730995 | 33.149839 | 33.994109 | 30.92 |
| 0 | Planned incoming peers | 7850 | 91.75 | 34.730995 | 33.907531 | 34.364670 | 28.00 |
| 0 | Delivered incoming peers | 7850 | 89.46 | 34.730995 | 33.963849 | 34.386051 | 26.87 |
| 0 | Physical one-hop peers | 7850 | 99.75 | 34.730995 | 29.655729 | 32.897697 | 44.14 |
| 1 | All network peers | 7818 | 99.97 | 34.786640 | 11.083094 | 28.039628 | 49.09 |
| 1 | Same formation | 7818 | 97.01 | 34.786640 | 33.504860 | 34.309411 | 32.02 |
| 1 | Planned incoming peers | 7818 | 96.38 | 34.786640 | 34.335751 | 34.617715 | 27.73 |
| 1 | Delivered incoming peers | 7818 | 94.10 | 34.786640 | 34.380086 | 34.633119 | 26.50 |
| 1 | Physical one-hop peers | 7818 | 99.87 | 34.786640 | 29.978635 | 33.590279 | 46.07 |

## Formation-level global-pool comparison

| Formation | Lag | Cases | Receiver pooled RMSE | Oracle pooled RMSE | Min-trace pooled RMSE |
| --- | --- | ---: | ---: | ---: | ---: |
| 1 | 0 | 1315 | 19.494156 | 7.476685 | 23.250843 |
| 1 | 1 | 1312 | 19.500466 | 7.294166 | 24.043021 |
| 2 | 0 | 1474 | 13.349694 | 6.984547 | 19.935894 |
| 2 | 1 | 1474 | 13.349694 | 6.867242 | 20.915609 |
| 3 | 0 | 1541 | 16.637272 | 5.583181 | 16.687384 |
| 3 | 1 | 1533 | 16.653354 | 5.696694 | 17.874941 |
| 4 | 0 | 1156 | 32.819468 | 9.853677 | 20.154239 |
| 4 | 1 | 1150 | 32.886685 | 10.015630 | 21.076056 |
| 5 | 0 | 1166 | 50.215719 | 11.478596 | 25.941825 |
| 5 | 1 | 1156 | 50.405806 | 11.448998 | 26.277733 |
| 6 | 0 | 1198 | 58.463069 | 20.606777 | 48.963328 |
| 6 | 1 | 1193 | 58.573599 | 20.561471 | 49.680838 |

The quantities are pooled sqrt(mean squared matched-position error), not official mean per-cell RMSE, OSPA or a counterfactual tracking trajectory. Cases repeat targets across nodes/time and are not independent samples. Same labels are required; no truth-based cross-label association is used.

Only emitted component means/covariances are inspected. Hidden components and non-emitted labels may contain other information. A favorable oracle does not show calibrated, available source selection; an unfavorable output comparison does not prove the full network posterior has no useful information.

Assignment readback: maximum discrepancy from saved official per-cell RMSE 1.42e-14 m; original-solver fallbacks 0. Full records remain in the local MAT; summary CSV preserves all formation/time-layer/source-pool aggregates.
