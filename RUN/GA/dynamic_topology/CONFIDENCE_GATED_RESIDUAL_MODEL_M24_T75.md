# Support-gated residual directed KLA model

- Model: `RUN/GA/dynamic_topology/models/confidence_gated_residual_m24_t75.mat`
- Training datasets: `RUN/GA/dynamic_topology/cache/directed_teacher_v2_m24_hard_seed7_t75.mat`, `RUN/GA/dynamic_topology/cache/directed_teacher_v2_m24_hard_seed17_t75.mat`
- Calibration datasets: `RUN/GA/dynamic_topology/cache/directed_teacher_v2_m24_hard_seed11_t75.mat`
- Training examples: 4416
- Calibration examples: 2208
- Training receiver groups: 48
- Calibration receiver groups: 24
- Training scenario/seed/time blocks: 2
- Calibration scenario/seed/time blocks: 1
- Scale-invariant action features: 32
- Ridge lambda: 1
- Nominal quantile parameter (not a coverage guarantee): 0.950
- Calibration mode: `selected-action`
- Calibration unit: `scenario-seed-time-block-max`
- Registered overestimate correction: 1.401135
- Simultaneous-action diagnostic correction: 3.895493
- Maximum calibration harmful-override fraction: 0.00%
- Empirical harmful-override correction escalated: `0`
- Calibration in-support actions: 82.25%

## Calibration selection

- Receiver override fraction: 0.00%
- Harmful override fraction: 0.00%
- Mean selected residual: 0.00%
- Minimum selected residual: 0.00%
- Oracle residual capture: 0.00%

## Evidence boundary

Ground truth is used only by the offline teacher labels. The empirical calibration result uses one complete M24 scenario/seed/time block. It is a block-level worst-error check, not a 95% population-coverage or unconditional closed-loop OSPA guarantee. X36 and new seeds must remain excluded from fitting and threshold selection.
