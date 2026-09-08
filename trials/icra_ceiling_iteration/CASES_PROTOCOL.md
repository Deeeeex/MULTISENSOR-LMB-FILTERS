# Frozen transfer to all cached synthetic mechanism cases

Prepared while the nine-sequence ECR development run is ongoing, before any
ECR synthetic outcome. Keep seeds 2901--2920 and all three existing cases,
with all cached model, measurement, truth, graph and loss uniforms unchanged.
Do not create marks from ground truth: every measurement, including clutter,
has mark one. All ECR variants therefore reduce to the same ECR-A rule.

Reuse the exact frozen local-update adapter, source eligibility, spatial
pool, age parameters and existence constraint. No threshold or candidate
formula changes. Extend the existing mixture-preserving packet by one
float64 per label, retain the fixed 16 KiB wire packet, and assert capacity.
Update local support on every local measurement step and never relay it as
fresh support. Preserve the original direct-time ownership.

Tracking receives measurements, model and link inputs only. Move the
existing score calls after the completed output trajectory. Compare all
60 runs against cached ER, no-age, CR/CGR and external baselines, preserving
each seed. Recompute errors and acquisition statistics independently. Record
the extra payload bytes and fixed packet equality; no runtime superiority
claim is inferred from the local Hungarian acceleration.
