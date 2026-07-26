# Directed-routing generalization audit

- Model: `RUN/GA/dynamic_topology/models/directed_routing_knn_m24_t75.mat`
- Snapshot: `t=75`
- Training examples: 552
- kNN neighbour count: 40
- Truth read by audit or policy: `0`

| Query | Nearest median | k=40 median | Any feature out of range | Selected receivers | Source card. shift | Receiver card. shift |
|:--|--:|--:|--:|--:|--:|--:|
| M24 training, leave-self-out | 1.165 | 4.216 | — | — | — | — |
| m24-hard seed 17 | 2.513 | 4.165 | 63.4% | 14/24 | -0.04 sigma | -0.04 sigma |
| x36-clean-scale seed 7 | 15.067 | 15.586 | 100.0% | 18/36 | 10.52 sigma | 10.52 sigma |
| x36-clean-scale seed 17 | 15.489 | 16.042 | 100.0% | 16/36 | 10.78 sigma | 10.78 sigma |

## Interpretation boundary

Distances are Euclidean in the exact standardized feature space consumed by the stored kNN model. A range violation means at least one query feature lies outside the elementwise minimum/maximum of the training examples; it is a support-shift diagnostic, not a formal density estimate. The audit uses posterior, geometry and link state only and does not establish closed-loop tracking quality.
