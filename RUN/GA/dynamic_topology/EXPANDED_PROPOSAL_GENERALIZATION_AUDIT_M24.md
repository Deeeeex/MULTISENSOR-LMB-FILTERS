# Expanded proposal-head generalization audit: M24

- Generated: 2026-07-29 15:45:20
- Audit contract: `rolling-safe-expanded-proposal-generalization-audit-m24-v1`
- Audit generation commit: `5c9442d82137c2b6db31ed3cc6fa17c5cc45d0f2`
- Model SHA-256: `3633d1eeaee7dbd4b00a538f553521b940ac19afbd8d56daedec2f8eca941a0b`
- Training-state capture: `54 / 54 (100.000%)`
- Training target-graph recall: `123 / 210 (58.571%)`
- Training mean best edge F1: `1.000000`
- Training mean distinct proposals: `2.814815`
- LOSO state capture: `2 / 54 (3.704%)`
- LOSO target-graph recall: `2 / 210 (0.952%)`
- State-capture generalization gap: `96.296 pp`
- Seed-specific generalization failure: `1`
- More capacity authorized: `0`
- Structured set objective / more seed diversity required: `1 / 1`
- Critic/X36 authorized: `0 / 0`
- Pass: `1`
- Evidence boundary: This is a development diagnosis on the model training states and the already-inspected LOSO folds. It localizes failure to cross-seed generalization; it is not held-out validation and does not establish tracking benefit.
