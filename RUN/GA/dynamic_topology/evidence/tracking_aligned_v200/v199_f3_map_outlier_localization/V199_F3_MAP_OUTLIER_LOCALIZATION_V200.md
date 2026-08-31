# V200 V199 F3 MAP-outlier localization

- Preset / seed: `x36-formation-fov / 211`
- Candidate-minus-static RMSE threshold: `50.0`
- Localized cells: `3`

| t | Sensor | RMSE excess | MAP C/R/T | Candidate label | Matched error | Static label/error | Candidate top/nearest component error | Nearest truth/error/occupancy | Static same-label `r` / MAP |
|--:|--:|--:|:--:|:--|--:|:--|:--|:--|:--|
| 76 | 15 | +138.928 | 20/20/24 | `[19,13]` | 659.253 | `[0,0]` / NaN | 659.253 / 659.253 | 17 / 654.557 / 1 | 0.328042 / no |
| 78 | 15 | +146.365 | 20/18/24 | `[19,13]` | 707.884 | `[0,0]` / NaN | 707.884 / 707.884 | 17 / 697.452 / 2 | 0.321514 / no |
| 79 | 16 | +154.572 | 20/17/24 | `[19,13]` | 732.410 | `[0,0]` / NaN | 732.410 / 732.410 | 17 / 718.887 / 2 | 0.318299 / no |

## Component details

- `t=76, sensor=15, truth target=24`: candidate `[19,13]`, existence `0.328042`, components `1`, top `#1 / w=1.000000 / d=659.253`, nearest `#1 / w=1.000000 / d=659.253`; static same-label present `1`, existence `0.328042`, top `#1 / w=1.000000 / d=659.253`; candidate estimates nearest to truth target 17: `[19,13] / 654.557`.
  - Cardinality candidate/static: expected `19.853 / 19.740`, MAP `20 / 20`, MAP probability `0.359690 / 0.345732`.
  - Candidate-only MAP labels: `[19,13] (r_c=0.328042, r_s=0.328042)`.
  - Static-only MAP labels: `[31,24] (r_c=0.256445, r_s=0.964337)`.
- `t=78, sensor=15, truth target=24`: candidate `[19,13]`, existence `0.321514`, components `1`, top `#1 / w=1.000000 / d=707.884`, nearest `#1 / w=1.000000 / d=707.884`; static same-label present `1`, existence `0.321514`, top `#1 / w=1.000000 / d=707.884`; candidate estimates nearest to truth target 17: `[25,20] / 9.457`, `[19,13] / 697.452`.
  - Cardinality candidate/static: expected `19.666 / 18.428`, MAP `20 / 18`, MAP probability `0.351888 / 0.335527`.
  - Candidate-only MAP labels: `[13,9] (r_c=0.995328, r_s=0.164396)`, `[7,5] (r_c=0.973936, r_s=0.295689)`, `[19,13] (r_c=0.321514, r_s=0.321514)`.
  - Static-only MAP labels: `[31,24] (r_c=0.197463, r_s=0.493175)`.
- `t=79, sensor=16, truth target=24`: candidate `[19,13]`, existence `0.318299`, components `1`, top `#1 / w=1.000000 / d=732.410`, nearest `#1 / w=1.000000 / d=732.410`; static same-label present `1`, existence `0.318299`, top `#1 / w=1.000000 / d=732.410`; candidate estimates nearest to truth target 17: `[25,20] / 8.631`, `[19,13] / 718.887`.
  - Cardinality candidate/static: expected `19.669 / 17.342`, MAP `20 / 17`, MAP probability `0.357543 / 0.406248`.
  - Candidate-only MAP labels: `[7,5] (r_c=0.996205, r_s=0.029857)`, `[13,9] (r_c=0.931291, r_s=0.040344)`, `[19,13] (r_c=0.318299, r_s=0.318299)`.
  - Static-only MAP labels: none.

## Evidence boundary

V200 reads current-time truth only after V199 has completed, to identify the MAP estimate responsible for each recorded F3 RMSE spike. It neither selects an online action nor supplies a feature to the future controller. The static and V199 snapshot readouts are required to reproduce the recorded per-cell RMSE exactly.
