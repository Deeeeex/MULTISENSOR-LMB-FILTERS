# V177 recursive-rollout univariate rank rules

- Rules / training-safe rules: `74 / 8`
- Heldout gate passed: `0`

| Feature | Direction | Train harmful | Train E/R/minR | Heldout harmful | Heldout E/R/minR |
|:--|:--|--:|:--|--:|:--|
| source_position_trace_normalized | minimum | 0 | +0.542/+3.321/+0.178 | 6 | -1.691/-11.066/-3.207 |
| confidence_disagreement_score | maximum | 0 | +42.846/+1.110/+0.141 | 3 | +44.280/-3.032/-1.406 |
| receiver_present | minimum | 0 | +42.797/+0.716/+0.098 | 5 | +44.071/-3.891/-1.094 |
| receiver_existence | minimum | 0 | +42.797/+0.716/+0.098 | 5 | +44.071/-3.891/-1.094 |
| receiver_bayes_risk | maximum | 0 | +42.797/+0.716/+0.098 | 5 | +44.071/-3.891/-1.094 |
| receiver_evidence_quality | minimum | 0 | +42.797/+0.716/+0.098 | 5 | +44.071/-3.891/-1.094 |
| log_mahalanobis_disagreement | minimum | 0 | +42.797/+0.716/+0.098 | 5 | +44.071/-3.891/-1.094 |
| receiver_source_compatibility | minimum | 0 | +42.797/+0.716/+0.098 | 5 | +44.071/-3.891/-1.094 |

## Evidence boundary

V177 is an opened univariate-rule screen on V176 rollout states. Each rule chooses one shortlisted source-label action by maximizing or minimizing one current observable feature, with label/source keys used only for deterministic tie-breaking. The rule is frozen using the six t=78 cells before the six t=79 cells are read. Current truth supplies action-value targets and rule selection, so this is a same-seed development diagnostic, not validation.
