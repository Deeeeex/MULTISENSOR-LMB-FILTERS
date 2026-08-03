# Dynamic-topology hard-scene difficulty audit

- Seeds: `[41 43 47 53 59]`
- All validations passed: `1`

| Preset | Sensors | Targets | Seed | Blackout | Focus blackout | Max target blackout | Longest blackout (steps) | Single formation | Multi formation | Focus handovers | Visible sensors | Expected detections | Target load / sensor | Cross-group close | Ownership entropy | Blockage overlap | Target sep. (m) | Sensor-target sep. (m) |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| m24-formation-fov-relay | 24 | 16 | 41 | 0.005 | 0.001 | 0.075 | 12 | 0.442 | 0.553 | 40 | 7.86 | 5.70 | 5.24 | 1.000 | 0.996 | 0.656 | 12.34 | 42.68 |
| x36-formation-fov-relay | 36 | 24 | 41 | 0.005 | 0.004 | 0.069 | 6 | 0.401 | 0.594 | 88 | 7.51 | 5.45 | 5.01 | 1.000 | 0.989 | 0.656 | 14.77 | 35.65 |
| m24-formation-fov-relay | 24 | 16 | 43 | 0.005 | 0.001 | 0.075 | 12 | 0.446 | 0.549 | 40 | 7.87 | 5.70 | 5.24 | 1.000 | 0.995 | 0.656 | 12.34 | 43.95 |
| x36-formation-fov-relay | 36 | 24 | 43 | 0.007 | 0.006 | 0.087 | 7 | 0.410 | 0.583 | 88 | 7.52 | 5.46 | 5.02 | 1.000 | 0.990 | 0.656 | 14.77 | 36.64 |
| m24-formation-fov-relay | 24 | 16 | 47 | 0.006 | 0.001 | 0.081 | 12 | 0.439 | 0.556 | 40 | 7.87 | 5.70 | 5.25 | 1.000 | 0.995 | 0.656 | 12.34 | 41.24 |
| x36-formation-fov-relay | 36 | 24 | 47 | 0.008 | 0.006 | 0.100 | 9 | 0.411 | 0.581 | 88 | 7.53 | 5.46 | 5.02 | 1.000 | 0.989 | 0.656 | 14.77 | 34.66 |
| m24-formation-fov-relay | 24 | 16 | 53 | 0.005 | 0.001 | 0.069 | 11 | 0.429 | 0.566 | 40 | 7.83 | 5.67 | 5.22 | 1.000 | 0.995 | 0.656 | 12.34 | 42.25 |
| x36-formation-fov-relay | 36 | 24 | 53 | 0.006 | 0.004 | 0.081 | 8 | 0.389 | 0.604 | 88 | 7.50 | 5.44 | 5.00 | 1.000 | 0.989 | 0.656 | 14.77 | 33.61 |
| m24-formation-fov-relay | 24 | 16 | 59 | 0.004 | 0.001 | 0.062 | 10 | 0.427 | 0.568 | 40 | 7.84 | 5.68 | 5.22 | 1.000 | 0.995 | 0.656 | 12.34 | 36.31 |
| x36-formation-fov-relay | 36 | 24 | 59 | 0.004 | 0.004 | 0.062 | 5 | 0.390 | 0.605 | 88 | 7.47 | 5.42 | 4.98 | 1.000 | 0.989 | 0.656 | 14.77 | 33.43 |

Blackout, single-formation, and multi-formation values are fractions of active target-time samples. Visible sensors and expected detections are per active target in the focus window; target load is visible targets per sensor-time in that window. This audit does not replace a tracking-filter stability or performance run.
