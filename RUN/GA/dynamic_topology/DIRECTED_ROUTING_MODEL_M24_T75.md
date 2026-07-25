# Directed routing model: M24-HARD t=75 checkpoint

- Dataset: `RUN/GA/dynamic_topology/cache/directed_teacher_m24_hard_seed7_t75.mat`
- Model: `RUN/GA/dynamic_topology/models/directed_routing_knn_m24_t75.mat`
- Examples: 552 directed physical links
- Receivers: 24
- Features: 21 deployment-observable values
- k nearest neighbours: 40
- Predicted-gain threshold: 0.0000
- Minimum positive-neighbour confidence: 0.70

## Receiver-held-out cross-validation

- Mean selected receiver gain: 15.21%
- Minimum selected receiver gain: 0.00%
- Harmful selected receivers: 0.00%
- Receivers selecting a source: 70.83%
- Oracle positive-gain capture: 42.63%

## Boundary

Ground truth is used only by the privileged teacher to produce training labels. Inference features and routing decisions do not access target truth. Receiver-held-out folds prevent edges entering one receiver from appearing in both training and validation, but this checkpoint is not seed- or time-held-out.
