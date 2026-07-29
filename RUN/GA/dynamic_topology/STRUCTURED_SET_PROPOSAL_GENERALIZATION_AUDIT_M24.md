# Structured set-proposal generalization audit: M24

- Generated: 2026-07-29 17:00:33
- Contract: `rolling-safe-structured-set-generalization-audit-m24-v1`
- Generation commit: `8f38c0412c95ceeff411b4b1d2c1160ab989518b`
- Model SHA-256: `9df630eba89c2da790514684793cf92e9455bc694b418791d51940851fdcf89d`
- Final sampled-bank training loss: `0.570087`
- Training-state capture: `0 / 81 (0.000%)`
- Training graph recall: `0 / 309 (0.000%)`
- Training mean best edge F1 / distinct: `0.255144 / 4.197531`
- LOSO state capture: `0 / 81 (0.000%)`
- State-capture gap: `0.000 pp`
- Seed-specific generalization failure: `0`
- Sampled-objective/exact-projector mismatch: `1`
- Exact-oracle hard-negative mining required: `1`
- More capacity authorized: `0`
- Return/critic/X36 authorized: `0 / 0 / 0`
- Pass: `1`
- Evidence boundary: This development diagnosis replays the final all-seed model on its training states through the same exact top-16 projector used in LOSO evaluation. It distinguishes training-set projector capture from cross-seed generalization; it is not held-out validation or tracking-benefit evidence.
