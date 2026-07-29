# Proposal-distillation dataset preflight: M24

- Generated: 2026-07-29 15:06:36
- Audit contract: `rolling-safe-proposal-distillation-audit-m24-v1`
- Audit generation commit: `d13365b05e161984a5d1c1747c3861fea1cce0c1`
- Dataset SHA-256: `ce815847e6e540e379934e2f09bbdb4f461cc2b4c7dc4ac918f35ff56feccf06`
- Dataset generation commit: `30d5ca3fcfe8a46aa37583c7a55cebe426c8e752`
- Source shard-set SHA-256: `5faa374e11428c663db949bc0db30d6de71f3b5d1ca75c8dedf1c2e210fa9409`
- Source shards verified: `6 / 6`
- State blocks / teacher targets: `54 / 210`
- Target counts for `[00 90 91 92]`: `[54 54 51 51]`
- Unique target / behavior / feature states: `210 / 54 / 54`
- Feature shape: `432 x 42` per state
- Varying feature columns: `35 / 42`
- Positive teacher edges / prevalence: `630 / 0.006944`
- Behavior/feature truth used: `0 / 0`
- Teacher truth/future used: `1 / 0`
- Behavior repair-free / feasible / B3: `1 / 1 / 1`
- Pass: `1`
- Evidence boundary: This preflight certifies a 54-state M24 development dataset collected under truth-free behavior and exact offline alignment of 210 current-truth safe teacher graphs. It does not establish proposal capture, value return, M24 benefit, generalization, critic accuracy or X36 benefit.
