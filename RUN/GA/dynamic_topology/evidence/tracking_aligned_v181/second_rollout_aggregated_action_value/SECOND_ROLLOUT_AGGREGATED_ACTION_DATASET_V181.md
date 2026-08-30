# V181 second rollout-aggregated action dataset

- Features / rows / cells: `37 / 4817 / 42`
- Duplicate V180 t=78 cells excluded: `6`

| Split | Cells | Rows | Joint-positive | Safe cells |
|:--|--:|--:|--:|--:|
| training | 30 | 3452 | 1097 | 30 |
| calibration | 6 | 679 | 224 | 6 |
| heldout | 6 | 686 | 222 | 6 |

## Evidence boundary

V181 is the second opened seed-211 policy-iteration dataset. It retains the V178 static and V176 t=78 training cells, promotes the V176 t=79 cells to training only after V179 evaluation, and keeps the original V166 F3 calibration cells unchanged. V180 t=78 is excluded because it is feature-identical to V176 t=78; only V180 t=79 is held out. Features remain present-time and truth-free, while current truth supplies immediate action targets. This same-seed DAgger-style update supports method development only.
