# Dynamic-topology hard-scene difficulty audit

- Seeds: `[41 43 47 53 59]`
- All validations passed: `1`

| Preset | Sensors | Targets | Seed | Blackout | Single formation | Multi formation | Focus handovers | Visible sensors | Expected detections | Target load / sensor | Cross-group close | Ownership entropy | Blockage overlap |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| m24-hard | 24 | 16 | 41 | 0.030 | 0.352 | 0.618 | 45 | 17.30 | 13.56 | 11.53 | 0.667 | 0.978 | 0.765 |
| x36-matched | 36 | 24 | 41 | 0.004 | 0.313 | 0.683 | 69 | 14.59 | 11.01 | 9.73 | 0.837 | 1.000 | 0.733 |
| x36-clean-scale | 36 | 24 | 41 | 0.000 | 0.094 | 0.906 | 69 | 23.70 | 17.91 | 15.80 | 0.837 | 1.000 | 0.733 |
| m24-hard | 24 | 16 | 43 | 0.031 | 0.352 | 0.617 | 43 | 17.30 | 13.55 | 11.53 | 0.667 | 0.978 | 0.765 |
| x36-matched | 36 | 24 | 43 | 0.004 | 0.320 | 0.676 | 67 | 14.61 | 11.03 | 9.74 | 0.837 | 1.000 | 0.733 |
| x36-clean-scale | 36 | 24 | 43 | 0.000 | 0.096 | 0.904 | 67 | 23.71 | 17.92 | 15.81 | 0.837 | 1.000 | 0.733 |
| m24-hard | 24 | 16 | 47 | 0.030 | 0.349 | 0.620 | 44 | 17.31 | 13.56 | 11.54 | 0.667 | 0.977 | 0.765 |
| x36-matched | 36 | 24 | 47 | 0.004 | 0.321 | 0.675 | 69 | 14.62 | 11.04 | 9.75 | 0.837 | 1.000 | 0.733 |
| x36-clean-scale | 36 | 24 | 47 | 0.000 | 0.096 | 0.904 | 69 | 23.72 | 17.92 | 15.81 | 0.837 | 1.000 | 0.733 |
| m24-hard | 24 | 16 | 53 | 0.028 | 0.351 | 0.621 | 43 | 17.29 | 13.55 | 11.53 | 0.667 | 0.978 | 0.765 |
| x36-matched | 36 | 24 | 53 | 0.004 | 0.310 | 0.686 | 69 | 14.58 | 11.01 | 9.72 | 0.837 | 0.999 | 0.733 |
| x36-clean-scale | 36 | 24 | 53 | 0.000 | 0.093 | 0.907 | 69 | 23.70 | 17.90 | 15.80 | 0.837 | 0.999 | 0.733 |
| m24-hard | 24 | 16 | 59 | 0.029 | 0.348 | 0.622 | 46 | 17.29 | 13.54 | 11.53 | 0.667 | 0.982 | 0.765 |
| x36-matched | 36 | 24 | 59 | 0.004 | 0.308 | 0.689 | 69 | 14.56 | 10.99 | 9.71 | 0.837 | 1.000 | 0.733 |
| x36-clean-scale | 36 | 24 | 59 | 0.000 | 0.093 | 0.907 | 69 | 23.68 | 17.89 | 15.79 | 0.837 | 1.000 | 0.733 |

Blackout, single-formation, and multi-formation values are fractions of active target-time samples. Visible sensors and expected detections are per active target in the focus window; target load is visible targets per sensor-time in that window. This audit does not replace a tracking-filter stability or performance run.
