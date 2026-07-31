# Same-hardware formation-FoV scale calibration

- Generated: `2026-08-01 00:55:36`
- Commit: `776ea951fbc32740a0b89496d45d0e32d3699c1f`
- Reference: `m24-formation-fov`
- Hardware profile: `formation-shared-150deg-r385-q300-v1`
- Total FoV / range: `150 deg / 385 m`
- Clutter space: `uniform-global-box2100-c4-v1`
- Hardware contract passed: `1`
- Seeds: `[41 43 47 53 59]`
- Mean / paired error caps: `1.0% / 3.0%`
- Calibration gate passed: `1`

| Preset | Sensors | Targets | Seed | Visible support | Expected detections (diagnostic) | Sensor load | Blackout | Single | Multi |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| x36-formation-fov | 36 | 24 | 41 | 0.036% | -7.463% | 0.036% | 0.000 | 0.142 | 0.858 |
| x48-formation-fov | 48 | 32 | 41 | -0.244% | -9.798% | -0.244% | 0.000 | 0.194 | 0.806 |
| x36-formation-fov | 36 | 24 | 43 | 0.147% | -7.378% | 0.147% | 0.000 | 0.146 | 0.854 |
| x48-formation-fov | 48 | 32 | 43 | -0.136% | -9.729% | -0.136% | 0.000 | 0.200 | 0.800 |
| x36-formation-fov | 36 | 24 | 47 | 0.187% | -7.348% | 0.187% | 0.000 | 0.147 | 0.853 |
| x48-formation-fov | 48 | 32 | 47 | -0.090% | -9.693% | -0.090% | 0.001 | 0.199 | 0.800 |
| x36-formation-fov | 36 | 24 | 53 | -0.005% | -7.442% | -0.005% | 0.000 | 0.140 | 0.860 |
| x48-formation-fov | 48 | 32 | 53 | -0.264% | -9.756% | -0.264% | 0.000 | 0.189 | 0.811 |
| x36-formation-fov | 36 | 24 | 59 | -0.083% | -7.512% | -0.083% | 0.000 | 0.138 | 0.862 |
| x48-formation-fov | 48 | 32 | 59 | -0.361% | -9.839% | -0.361% | 0.000 | 0.185 | 0.815 |

Only sensor load enters the geometry-matching gate. Visible support is a derived check; expected detection differences are diagnostic and excluded from the gate.

## Aggregate geometry gate

| Preset | Max mean error | Max paired error | Gate |
|:--|--:|--:|:--:|
| x36-formation-fov | 0.056% | 0.187% | 1 |
| x48-formation-fov | 0.219% | 0.361% | 1 |

## Interpretation boundary

The opened seeds calibrate scene geometry only. Passing proves that the sole independent geometry gate, local per-sensor sensing load, is matched while every sensor-hardware and clutter-space parameter remains identical under a fixed 150-degree total FoV and formation-shared boresight contract. Visible support is a derived sanity check because the target-to-sensor ratio is fixed. Expected detection count is diagnostic scale pressure, not an equivalence gate. This audit does not establish tracking gain, policy generalization, or an X36/X48 validation claim.
