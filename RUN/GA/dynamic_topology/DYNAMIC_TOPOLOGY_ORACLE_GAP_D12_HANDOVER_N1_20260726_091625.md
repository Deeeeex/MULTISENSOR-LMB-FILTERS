# Dynamic-topology oracle-gap screen

- Preset: `d12-handover`
- Seeds: `7`
- Generated: 2026-07-26 09:16:25
- Decision status: `dynamic-attribution-gate-failed`

- Focus window: `handover`, steps `[1 3]`

- Analysis window: steps `[1 3]`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Undirected edges | Directed routes | Attempts | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Directed fixed index control (w=0.50) | 74.4548 | 100.0000 | 0.4184 | 49.4026 | 2.1111 | 58392 | 58392 | 9.00 | 12.00 | 36 | 0.0000 | 0.0000 | 0.00 | 0.70 |
| Directed fixed cycle control (w=0.50) | 74.2965 | 100.0000 | 0.4256 | 49.7924 | 2.1111 | 60912 | 60912 | 12.00 | 12.00 | 36 | 0.0000 | 0.0000 | 0.00 | 0.67 |
| Directed round robin control (w=0.50) | 74.2049 | 100.0000 | 0.4227 | 49.7016 | 2.1111 | 61248 | 61248 | 10.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.67 |
| Directed physical round robin control (phase=1, w=0.50) | 44.5651 | 55.4301 | 0.2494 | 27.4120 | 1.0000 | 129792 | 129792 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.93 |
| Directed physical round robin control (phase=2, w=0.50) | 43.7929 | 54.2798 | 0.2550 | 27.3840 | 1.0000 | 155328 | 155328 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.96 |
| Directed physical round robin control (phase=3, w=0.50) | 51.4464 | 57.9229 | 0.2079 | 28.4283 | 1.1389 | 130464 | 130464 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.92 |
| Directed physical round robin control (phase=4, w=0.50) | 94.9684 | 100.0000 | 0.1313 | 27.9602 | 2.7500 | 84096 | 84096 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.80 |
| Directed physical round robin control (phase=5, w=0.50) | 99.4913 | 100.0000 | 0.0583 | 5.5556 | 2.9722 | 23616 | 23616 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.56 |
| Directed physical round robin control (phase=6, w=0.50) | 99.4913 | 100.0000 | 0.0583 | 5.5556 | 2.9722 | 23616 | 23616 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.56 |
| Directed physical round robin control (phase=7, w=0.50) | 99.4913 | 100.0000 | 0.0583 | 5.5556 | 2.9722 | 23616 | 23616 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.56 |
| Directed physical round robin control (phase=8, w=0.50) | 99.4913 | 100.0000 | 0.0583 | 5.5556 | 2.9722 | 23616 | 23616 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.56 |
| Directed physical round robin control (phase=9, w=0.50) | 99.4913 | 100.0000 | 0.0583 | 5.5556 | 2.9722 | 23616 | 23616 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.57 |
| Directed physical round robin control (phase=10, w=0.50) | 93.8525 | 100.0000 | 0.1478 | 17.0763 | 2.7500 | 86280 | 86280 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.60 |
| Directed physical round robin control (phase=11, w=0.50) | 77.8445 | 93.8875 | 0.2384 | 29.8076 | 2.1667 | 137184 | 137184 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.72 |
| Directed fixed index control (w=0.70) | 74.6467 | 100.0000 | 0.4246 | 49.4403 | 2.1111 | 61752 | 61752 | 9.00 | 12.00 | 36 | 0.0000 | 0.0000 | 0.00 | 0.68 |
| Directed fixed cycle control (w=0.70) | 74.3629 | 100.0000 | 0.4339 | 49.8532 | 2.1111 | 65280 | 65280 | 12.00 | 12.00 | 36 | 0.0000 | 0.0000 | 0.00 | 0.67 |
| Directed round robin control (w=0.70) | 74.2997 | 100.0000 | 0.4317 | 49.8062 | 2.1111 | 66456 | 66456 | 10.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.67 |
| Directed physical round robin control (phase=1, w=0.70) | 29.5893 | 32.8061 | 0.2119 | 14.0792 | 0.5556 | 139368 | 139368 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.93 |
| Directed physical round robin control (phase=2, w=0.70) | 28.1670 | 30.8424 | 0.1858 | 13.7515 | 0.5556 | 161376 | 161376 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.93 |
| Directed physical round robin control (phase=3, w=0.70) | 35.3166 | 48.6740 | 0.2209 | 24.7921 | 0.6944 | 133152 | 133152 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.92 |
| Directed physical round robin control (phase=4, w=0.70) | 83.7697 | 87.8313 | 0.0946 | 33.2902 | 2.1111 | 84096 | 84096 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.80 |
| Directed physical round robin control (phase=5, w=0.70) | 100.0000 | 100.0000 | 0.0196 | 0.0000 | 3.0000 | 23616 | 23616 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.56 |
| Directed physical round robin control (phase=6, w=0.70) | 100.0000 | 100.0000 | 0.0196 | 0.0000 | 3.0000 | 23616 | 23616 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.56 |
| Directed physical round robin control (phase=7, w=0.70) | 100.0000 | 100.0000 | 0.0196 | 0.0000 | 3.0000 | 23616 | 23616 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.56 |
| Directed physical round robin control (phase=8, w=0.70) | 100.0000 | 100.0000 | 0.0196 | 0.0000 | 3.0000 | 23616 | 23616 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.56 |
| Directed physical round robin control (phase=9, w=0.70) | 100.0000 | 100.0000 | 0.0196 | 0.0000 | 3.0000 | 23616 | 23616 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.56 |
| Directed physical round robin control (phase=10, w=0.70) | 89.8253 | 93.9139 | 0.0988 | 13.5450 | 2.5278 | 88128 | 88128 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.60 |
| Directed physical round robin control (phase=11, w=0.70) | 69.0475 | 79.8644 | 0.1560 | 19.8551 | 1.7222 | 147600 | 147600 | 11.00 | 12.00 | 36 | 1.0000 | 0.0000 | 0.00 | 0.71 |
| Directed reliability + source quality (c=0.25, w=0.70) | 28.6241 | 31.3518 | 0.1569 | 13.5424 | 0.5556 | 117360 | 117360 | 11.00 | 12.00 | 36 | 0.9333 | 0.0000 | 0.41 | 1.35 |

## Analysis-window route-attribution diagnostics

| Arm | Boundary-inclusive churn | Prefix receiver changes | Within-window receiver changes | Distinct maps | Different from fixed index | Receiver coverage | Unique senders / receiver | Cross-formation routes |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|
| Directed fixed index control (w=0.50) | 0.0000 | 0.0000 | 0.0000 | 1.00 | 0.0000 | 1.0000 | 1.00 | 0.0000 |
| Directed fixed cycle control (w=0.50) | 0.0000 | 0.0000 | 0.0000 | 1.00 | 0.5000 | 1.0000 | 1.00 | 0.0000 |
| Directed round robin control (w=0.50) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.6667 | 1.0000 | 3.00 | 0.0000 |
| Directed physical round robin control (phase=1, w=0.50) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.8889 | 1.0000 | 3.00 | 0.6667 |
| Directed physical round robin control (phase=2, w=0.50) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 1.0000 | 1.0000 | 3.00 | 0.7778 |
| Directed physical round robin control (phase=3, w=0.50) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.8889 | 1.0000 | 3.00 | 0.7778 |
| Directed physical round robin control (phase=4, w=0.50) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.8889 | 1.0000 | 3.00 | 0.7778 |
| Directed physical round robin control (phase=5, w=0.50) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.8889 | 1.0000 | 3.00 | 0.6667 |
| Directed physical round robin control (phase=6, w=0.50) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 1.0000 | 1.0000 | 3.00 | 0.7778 |
| Directed physical round robin control (phase=7, w=0.50) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.8889 | 1.0000 | 3.00 | 0.7778 |
| Directed physical round robin control (phase=8, w=0.50) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.8889 | 1.0000 | 3.00 | 0.7778 |
| Directed physical round robin control (phase=9, w=0.50) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.8889 | 1.0000 | 3.00 | 0.6667 |
| Directed physical round robin control (phase=10, w=0.50) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.8889 | 1.0000 | 3.00 | 0.6667 |
| Directed physical round robin control (phase=11, w=0.50) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.8889 | 1.0000 | 3.00 | 0.6667 |
| Directed fixed index control (w=0.70) | 0.0000 | 0.0000 | 0.0000 | 1.00 | 0.0000 | 1.0000 | 1.00 | 0.0000 |
| Directed fixed cycle control (w=0.70) | 0.0000 | 0.0000 | 0.0000 | 1.00 | 0.5000 | 1.0000 | 1.00 | 0.0000 |
| Directed round robin control (w=0.70) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.6667 | 1.0000 | 3.00 | 0.0000 |
| Directed physical round robin control (phase=1, w=0.70) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.8889 | 1.0000 | 3.00 | 0.6667 |
| Directed physical round robin control (phase=2, w=0.70) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 1.0000 | 1.0000 | 3.00 | 0.7778 |
| Directed physical round robin control (phase=3, w=0.70) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.8889 | 1.0000 | 3.00 | 0.7778 |
| Directed physical round robin control (phase=4, w=0.70) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.8889 | 1.0000 | 3.00 | 0.7778 |
| Directed physical round robin control (phase=5, w=0.70) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.8889 | 1.0000 | 3.00 | 0.6667 |
| Directed physical round robin control (phase=6, w=0.70) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 1.0000 | 1.0000 | 3.00 | 0.7778 |
| Directed physical round robin control (phase=7, w=0.70) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.8889 | 1.0000 | 3.00 | 0.7778 |
| Directed physical round robin control (phase=8, w=0.70) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.8889 | 1.0000 | 3.00 | 0.7778 |
| Directed physical round robin control (phase=9, w=0.70) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.8889 | 1.0000 | 3.00 | 0.6667 |
| Directed physical round robin control (phase=10, w=0.70) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.8889 | 1.0000 | 3.00 | 0.6667 |
| Directed physical round robin control (phase=11, w=0.70) | 1.0000 | 0.0000 | 1.0000 | 3.00 | 0.8889 | 1.0000 | 3.00 | 0.6667 |
| Directed reliability + source quality (c=0.25, w=0.70) | 0.9333 | 0.0000 | 0.8750 | 3.00 | 0.9444 | 1.0000 | 1.92 | 0.6667 |

When continuation is used, boundary-inclusive churn also contains the one-time transition from the common static prefix. Only the within-window receiver-change rate and distinct-map count establish that an arm actually changes routes during the evaluated window.

## Focus-window result

| Arm | Focus E-OSPA | Focus worst node | Focus attempted bytes | Focus route changes | Focus maps | Focus coverage | Different from fixed index | Focus cross-formation | Focus infeasible | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Directed fixed index control (w=0.50) | 74.4548 | 100.0000 | 58392 | 0.0000 | 1.00 | 1.0000 | 0.0000 | 0.0000 | 0.0000 | 0.4184 | 49.4026 | 2.1111 |
| Directed fixed cycle control (w=0.50) | 74.2965 | 100.0000 | 60912 | 0.0000 | 1.00 | 1.0000 | 0.5000 | 0.0000 | 0.0000 | 0.4256 | 49.7924 | 2.1111 |
| Directed round robin control (w=0.50) | 74.2049 | 100.0000 | 61248 | 1.0000 | 3.00 | 1.0000 | 0.6667 | 0.0000 | 0.0000 | 0.4227 | 49.7016 | 2.1111 |
| Directed physical round robin control (phase=1, w=0.50) | 44.5651 | 55.4301 | 129792 | 1.0000 | 3.00 | 1.0000 | 0.8889 | 0.6667 | 0.0000 | 0.2494 | 27.4120 | 1.0000 |
| Directed physical round robin control (phase=2, w=0.50) | 43.7929 | 54.2798 | 155328 | 1.0000 | 3.00 | 1.0000 | 1.0000 | 0.7778 | 0.0000 | 0.2550 | 27.3840 | 1.0000 |
| Directed physical round robin control (phase=3, w=0.50) | 51.4464 | 57.9229 | 130464 | 1.0000 | 3.00 | 1.0000 | 0.8889 | 0.7778 | 0.0000 | 0.2079 | 28.4283 | 1.1389 |
| Directed physical round robin control (phase=4, w=0.50) | 94.9684 | 100.0000 | 84096 | 1.0000 | 3.00 | 1.0000 | 0.8889 | 0.7778 | 0.0000 | 0.1313 | 27.9602 | 2.7500 |
| Directed physical round robin control (phase=5, w=0.50) | 99.4913 | 100.0000 | 23616 | 1.0000 | 3.00 | 1.0000 | 0.8889 | 0.6667 | 0.0000 | 0.0583 | 5.5556 | 2.9722 |
| Directed physical round robin control (phase=6, w=0.50) | 99.4913 | 100.0000 | 23616 | 1.0000 | 3.00 | 1.0000 | 1.0000 | 0.7778 | 0.0000 | 0.0583 | 5.5556 | 2.9722 |
| Directed physical round robin control (phase=7, w=0.50) | 99.4913 | 100.0000 | 23616 | 1.0000 | 3.00 | 1.0000 | 0.8889 | 0.7778 | 0.0000 | 0.0583 | 5.5556 | 2.9722 |
| Directed physical round robin control (phase=8, w=0.50) | 99.4913 | 100.0000 | 23616 | 1.0000 | 3.00 | 1.0000 | 0.8889 | 0.7778 | 0.0000 | 0.0583 | 5.5556 | 2.9722 |
| Directed physical round robin control (phase=9, w=0.50) | 99.4913 | 100.0000 | 23616 | 1.0000 | 3.00 | 1.0000 | 0.8889 | 0.6667 | 0.0000 | 0.0583 | 5.5556 | 2.9722 |
| Directed physical round robin control (phase=10, w=0.50) | 93.8525 | 100.0000 | 86280 | 1.0000 | 3.00 | 1.0000 | 0.8889 | 0.6667 | 0.0000 | 0.1478 | 17.0763 | 2.7500 |
| Directed physical round robin control (phase=11, w=0.50) | 77.8445 | 93.8875 | 137184 | 1.0000 | 3.00 | 1.0000 | 0.8889 | 0.6667 | 0.0000 | 0.2384 | 29.8076 | 2.1667 |
| Directed fixed index control (w=0.70) | 74.6467 | 100.0000 | 61752 | 0.0000 | 1.00 | 1.0000 | 0.0000 | 0.0000 | 0.0000 | 0.4246 | 49.4403 | 2.1111 |
| Directed fixed cycle control (w=0.70) | 74.3629 | 100.0000 | 65280 | 0.0000 | 1.00 | 1.0000 | 0.5000 | 0.0000 | 0.0000 | 0.4339 | 49.8532 | 2.1111 |
| Directed round robin control (w=0.70) | 74.2997 | 100.0000 | 66456 | 1.0000 | 3.00 | 1.0000 | 0.6667 | 0.0000 | 0.0000 | 0.4317 | 49.8062 | 2.1111 |
| Directed physical round robin control (phase=1, w=0.70) | 29.5893 | 32.8061 | 139368 | 1.0000 | 3.00 | 1.0000 | 0.8889 | 0.6667 | 0.0000 | 0.2119 | 14.0792 | 0.5556 |
| Directed physical round robin control (phase=2, w=0.70) | 28.1670 | 30.8424 | 161376 | 1.0000 | 3.00 | 1.0000 | 1.0000 | 0.7778 | 0.0000 | 0.1858 | 13.7515 | 0.5556 |
| Directed physical round robin control (phase=3, w=0.70) | 35.3166 | 48.6740 | 133152 | 1.0000 | 3.00 | 1.0000 | 0.8889 | 0.7778 | 0.0000 | 0.2209 | 24.7921 | 0.6944 |
| Directed physical round robin control (phase=4, w=0.70) | 83.7697 | 87.8313 | 84096 | 1.0000 | 3.00 | 1.0000 | 0.8889 | 0.7778 | 0.0000 | 0.0946 | 33.2902 | 2.1111 |
| Directed physical round robin control (phase=5, w=0.70) | 100.0000 | 100.0000 | 23616 | 1.0000 | 3.00 | 1.0000 | 0.8889 | 0.6667 | 0.0000 | 0.0196 | 0.0000 | 3.0000 |
| Directed physical round robin control (phase=6, w=0.70) | 100.0000 | 100.0000 | 23616 | 1.0000 | 3.00 | 1.0000 | 1.0000 | 0.7778 | 0.0000 | 0.0196 | 0.0000 | 3.0000 |
| Directed physical round robin control (phase=7, w=0.70) | 100.0000 | 100.0000 | 23616 | 1.0000 | 3.00 | 1.0000 | 0.8889 | 0.7778 | 0.0000 | 0.0196 | 0.0000 | 3.0000 |
| Directed physical round robin control (phase=8, w=0.70) | 100.0000 | 100.0000 | 23616 | 1.0000 | 3.00 | 1.0000 | 0.8889 | 0.7778 | 0.0000 | 0.0196 | 0.0000 | 3.0000 |
| Directed physical round robin control (phase=9, w=0.70) | 100.0000 | 100.0000 | 23616 | 1.0000 | 3.00 | 1.0000 | 0.8889 | 0.6667 | 0.0000 | 0.0196 | 0.0000 | 3.0000 |
| Directed physical round robin control (phase=10, w=0.70) | 89.8253 | 93.9139 | 88128 | 1.0000 | 3.00 | 1.0000 | 0.8889 | 0.6667 | 0.0000 | 0.0988 | 13.5450 | 2.5278 |
| Directed physical round robin control (phase=11, w=0.70) | 69.0475 | 79.8644 | 147600 | 1.0000 | 3.00 | 1.0000 | 0.8889 | 0.6667 | 0.0000 | 0.1560 | 19.8551 | 1.7222 |
| Directed reliability + source quality (c=0.25, w=0.70) | 28.6241 | 31.3518 | 117360 | 0.8750 | 3.00 | 1.0000 | 0.9444 | 0.6667 | 0.0000 | 0.1569 | 13.5424 | 0.5556 |

## Registered gate readout

- Constraint-eligible arms: `0`
- Best observed arm: ``
- Best observed focus E-OSPA: NaN
- Best observed gain vs static: NaN%
- Minimum practical tracking gain: 5.00%
- Best observed byte mismatch vs static: NaN%
- Oracle consensus gain: NaN%
- Oracle tracking gain: NaN%
- Analytic share of static-to-oracle gain: NaN
- Diagnostic reference dominated: `0`
- Attempted-byte mismatch: NaN%
- Recommendation: The candidate does not jointly pass the gain, per-seed, tail, byte, feasibility and within-window route-change gates against the strongest registered controls.

## Learned directed-routing readout

- Eligible learned directed arms: `0`
- Best learned directed arm: ``
- Focus E-OSPA: NaN
- Gain vs static: NaN%
- Gain vs local: NaN%
- Worst-node gain vs static: NaN%
- Worst-node gain vs local: NaN%
- Attempted bytes relative to static: NaN%
- Mean selected directed routes: NaN
- Zero infeasibility: `0`
- Strict tail-safe vs static: `0`
- Strict tail-safe vs local: `0`
- Passes registered mean-tracking gate: `0`
- Directed-routing status: `unavailable`

## Residual-versus-backbone readout

- Eligible residual arms: `0`
- Residual arm: ``
- Registered backbone: ``
- Residual focus E-OSPA: NaN
- Backbone focus E-OSPA: NaN
- Gain vs backbone: NaN%
- Worst-node gain vs backbone: NaN%
- Attempted bytes relative to backbone: NaN%
- Policy-time overhead: NaN s
- Mean learned override fraction: NaN
- Mean in-support candidate fraction: NaN
- Exact backbone match: `0`
- Positive gain on every seed: `0`
- Strict tail-safe vs backbone: `0`
- Within backbone attempted bytes (+2%): `0`
- Minimum practical incremental gain: NaN%
- Passes incremental-learning gate: `0`
- Residual-routing status: `unavailable`

## Dynamic-routing attribution readout

- Candidate arm: `Directed reliability + source quality (c=0.25, w=0.70)`
- Available candidates: `1`
- Candidates with complete matched controls: `1`
- Structurally eligible candidates: `1`
- Strong-control reference: `best per-seed registered fixed/scheduled control across weights`
- Weight-matched reference: `best per-seed weight-matched registered control (including physical round-robin when required)`
- Complete registered control set: `1`
- Candidate source weight: 0.70
- Weight-matched control set: `1`
- Candidate uses cross-formation routes: `1`
- Cross-formation scheduled control set: `1`
- Action support matched: `1`
- Gain vs weight-matched control: -1.6229%
- Gain vs weight-matched control by seed: `-1.6229`
- Positive matched-control gain on every seed: `0`
- Gain vs strongest control: -1.6229%
- Gain by seed: `-1.6229`
- Positive gain on every seed: `0`
- Strict tail-safe vs strongest control: `0`
- Maximum attempted-byte mismatch: 27.2754%
- Attempted bytes matched within 2%: `0`
- No more attempted bytes on every seed: `1`
- Strict tracking-byte Pareto on every seed: `0`
- Passes communication fairness: `0`
- Distinct maps on every seed: `1`
- Within-window changes on every seed: `1`
- Minimum difference from fixed index: 0.9444
- Complete receiver coverage: `1`
- Cross-formation routes observed: `1`
- Passes dynamic-attribution gate: `0`
- Dynamic-attribution status: `dynamic-attribution-gate-failed`

## Evidence limits

- This runner isolates topology: every active edge sends the same heavy posterior every step; event triggering and payload compression are disabled.
- A one- or three-seed run is a software/runtime screen, not a paper-level effect estimate. The registered screening gate needs at least 10 paired seeds; the held-out claim needs 30.
- Equal edge budgets do not guarantee equal bytes. The table therefore reports attempted payload bytes explicitly.
- Sparse directed routing is evaluated as a Pareto arm: it must use no more attempted payload bytes than static (within 2% accounting tolerance) and must beat both static and local on mean tracking. It is not required to match the static edge count.
- The static arm is selected by all-time geometry and link distance; it is not an exhaustive offline performance optimum.
- Exact one-step action enumeration is not a closed-loop upper bound. If it is dominated by a deployable arm, it cannot justify a learned teacher or an analytic-sufficiency claim.
