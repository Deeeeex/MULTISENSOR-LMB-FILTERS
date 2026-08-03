# Dynamic-topology hard-scene difficulty audit

- Seeds: `[41 43 47 53 59]`
- All validations passed: `1`

| Preset | Sensors | Targets | Seed | Blackout | Focus blackout | Max target blackout | Longest blackout (steps) | Single formation | Multi formation | Focus handovers | Visible sensors | Expected detections | Target load / sensor | Cross-group close | Ownership entropy | Blockage overlap |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| m24-formation-fov-convoy | 24 | 16 | 41 | 0.014 | 0.020 | 0.053 | 8 | 0.341 | 0.646 | 16 | 7.15 | 5.36 | 4.76 | 1.000 | 0.991 | 0.656 |
| x36-formation-fov-convoy | 36 | 24 | 41 | 0.011 | 0.016 | 0.055 | 7 | 0.308 | 0.681 | 24 | 7.72 | 5.74 | 5.15 | 1.000 | 0.992 | 0.656 |
| m24-formation-fov-convoy | 24 | 16 | 43 | 0.013 | 0.019 | 0.051 | 7 | 0.353 | 0.635 | 16 | 7.16 | 5.36 | 4.77 | 1.000 | 0.991 | 0.656 |
| x36-formation-fov-convoy | 36 | 24 | 43 | 0.015 | 0.021 | 0.062 | 8 | 0.321 | 0.665 | 24 | 7.73 | 5.75 | 5.16 | 1.000 | 0.991 | 0.656 |
| m24-formation-fov-convoy | 24 | 16 | 47 | 0.014 | 0.021 | 0.053 | 8 | 0.349 | 0.637 | 16 | 7.15 | 5.36 | 4.76 | 1.000 | 0.990 | 0.656 |
| x36-formation-fov-convoy | 36 | 24 | 47 | 0.013 | 0.019 | 0.062 | 8 | 0.313 | 0.674 | 24 | 7.73 | 5.74 | 5.15 | 1.000 | 0.991 | 0.656 |
| m24-formation-fov-convoy | 24 | 16 | 53 | 0.008 | 0.012 | 0.040 | 6 | 0.333 | 0.659 | 16 | 7.12 | 5.34 | 4.75 | 1.000 | 0.992 | 0.656 |
| x36-formation-fov-convoy | 36 | 24 | 53 | 0.008 | 0.012 | 0.044 | 6 | 0.299 | 0.693 | 24 | 7.72 | 5.73 | 5.14 | 1.000 | 0.992 | 0.656 |
| m24-formation-fov-convoy | 24 | 16 | 59 | 0.008 | 0.012 | 0.043 | 6 | 0.328 | 0.664 | 16 | 7.13 | 5.35 | 4.76 | 1.000 | 0.992 | 0.656 |
| x36-formation-fov-convoy | 36 | 24 | 59 | 0.009 | 0.013 | 0.047 | 6 | 0.285 | 0.705 | 24 | 7.71 | 5.73 | 5.14 | 1.000 | 0.992 | 0.656 |

Blackout, single-formation, and multi-formation values are fractions of active target-time samples. Visible sensors and expected detections are per active target in the focus window; target load is visible targets per sensor-time in that window. This audit does not replace a tracking-filter stability or performance run.
