# Same-hardware formation-FoV scale calibration

- Generated: `2026-08-01 14:47:08`
- Commit: `a8879ab42926e28a299a3b68ef878ad4149e6fda`
- Reference: `m24-formation-fov`
- Hardware profile: `formation-shared-120deg-r300-q300-v1`
- Total FoV / range: `120 deg / 300 m`
- Clutter space: `uniform-global-box2100-c4-v1`
- Hardware contract passed: `1`
- Seeds: `[41 43 47 53 59]`
- Mean / paired error caps: `1.0% / 3.0%`
- Calibration gate passed: `1`

| Preset | Sensors | Targets | Seed | Visible support | Expected detections (diagnostic) | Sensor load | Blackout | Focus blackout | Max target blackout | Longest blackout (steps) | Single | Multi |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| x36-formation-fov | 36 | 24 | 41 | -0.474% | -3.118% | -0.474% | 0.136 | 0.000 | 0.194 | 19 | 0.079 | 0.785 |
| x48-formation-fov | 48 | 32 | 41 | -0.218% | -3.931% | -0.218% | 0.144 | 0.000 | 0.225 | 23 | 0.076 | 0.780 |
| x36-formation-fov | 36 | 24 | 43 | -0.304% | -2.958% | -0.304% | 0.140 | 0.000 | 0.200 | 19 | 0.074 | 0.786 |
| x48-formation-fov | 48 | 32 | 43 | -0.162% | -3.892% | -0.162% | 0.147 | 0.000 | 0.237 | 24 | 0.074 | 0.779 |
| x36-formation-fov | 36 | 24 | 47 | -0.281% | -2.941% | -0.281% | 0.140 | 0.000 | 0.200 | 20 | 0.074 | 0.785 |
| x48-formation-fov | 48 | 32 | 47 | -0.231% | -3.960% | -0.231% | 0.146 | 0.000 | 0.237 | 24 | 0.075 | 0.779 |
| x36-formation-fov | 36 | 24 | 53 | -0.305% | -2.898% | -0.305% | 0.134 | 0.000 | 0.188 | 19 | 0.079 | 0.787 |
| x48-formation-fov | 48 | 32 | 53 | -0.077% | -3.733% | -0.077% | 0.141 | 0.000 | 0.225 | 23 | 0.078 | 0.781 |
| x36-formation-fov | 36 | 24 | 59 | -0.484% | -3.076% | -0.484% | 0.134 | 0.000 | 0.200 | 19 | 0.078 | 0.788 |
| x48-formation-fov | 48 | 32 | 59 | -0.199% | -3.849% | -0.199% | 0.142 | 0.000 | 0.225 | 23 | 0.075 | 0.783 |

Only sensor load enters the geometry-matching gate. Visible support is a derived check; expected detection differences are diagnostic and excluded from the gate.

## Aggregate geometry gate

| Preset | Max mean error | Max paired error | Gate |
|:--|--:|--:|:--:|
| x36-formation-fov | 0.370% | 0.484% | 1 |
| x48-formation-fov | 0.178% | 0.231% | 1 |

## Interpretation boundary

The opened seeds calibrate scene geometry only. Passing proves that the sole cross-scale matching statistic, local per-sensor sensing load, is matched while every sensor-hardware and clutter-space parameter remains identical under a fixed 120-degree total FoV, a 300 m hard sensing range, and the `formation-shared-scene-center` boresight contract. Visible support is a derived sanity check because the target-to-sensor ratio is fixed. Focus, per-target, and consecutive-blackout gates prevent the aggregate load match from hiding prolonged unobservable targets. Expected detection count is diagnostic scale pressure, not an equivalence gate. This audit does not establish tracking gain, policy generalization, or an X36/X48 validation claim.
