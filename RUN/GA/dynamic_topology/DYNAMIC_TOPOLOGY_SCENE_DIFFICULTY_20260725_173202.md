# Dynamic-topology hard-scene difficulty audit

- Seeds: `[7 17 27]`
- All validations passed: `1`

| Preset | Sensors | Targets | Seed | Blackout | Single formation | Multi formation | Focus handovers | Cross-group close | Ownership entropy | Blockage overlap |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| d12-hard | 12 | 12 | 7 | 0.000 | 0.809 | 0.191 | 18 | 0.474 | 1.000 | 0.000 |
| m24-hard | 24 | 16 | 7 | 0.030 | 0.351 | 0.620 | 44 | 0.667 | 0.978 | 0.765 |
| x36-hard | 36 | 24 | 7 | 0.031 | 0.382 | 0.587 | 69 | 0.837 | 1.000 | 0.733 |
| d12-hard | 12 | 12 | 17 | 0.000 | 0.806 | 0.194 | 18 | 0.474 | 1.000 | 0.000 |
| m24-hard | 24 | 16 | 17 | 0.028 | 0.349 | 0.622 | 46 | 0.667 | 0.980 | 0.765 |
| x36-hard | 36 | 24 | 17 | 0.029 | 0.371 | 0.599 | 69 | 0.837 | 1.000 | 0.733 |
| d12-hard | 12 | 12 | 27 | 0.000 | 0.807 | 0.193 | 18 | 0.474 | 1.000 | 0.000 |
| m24-hard | 24 | 16 | 27 | 0.029 | 0.351 | 0.620 | 46 | 0.667 | 0.981 | 0.765 |
| x36-hard | 36 | 24 | 27 | 0.029 | 0.374 | 0.597 | 70 | 0.837 | 0.999 | 0.733 |

Blackout, single-formation, and multi-formation values are fractions of active target-time samples. This audit does not replace a tracking-filter stability or performance run.
