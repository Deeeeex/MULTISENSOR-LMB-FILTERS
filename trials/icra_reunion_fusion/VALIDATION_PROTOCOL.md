# Frozen outcome validation v4

Registered 2026-09-08 after all development results and before new seeds.
This is outcome validation of a conditional fusion finding, not a claim
that the original development gate passed. Every new development candidate
failed at least one strict screen. The v2 existence-only arm improves the
lineage baseline's OSPA/departure false cost without changing common-target
RMSE, but repeats its seven no-new false estimates. Confirmation removes
those extra false estimates but damages discovery in the churn scene.
These failures remain in the paper. Stop method/threshold/age tuning.

## Inputs fixed before validation

Use all three scene families, seeds 2901--2920 inclusive (20 paired episodes
per family). Before generating measurements, perturb each robot's entire
path by one fixed independent XY offset uniform in [-0.6,0.6] m, using RNG
seed+200000. Recompute physical radio components and each component's MST.
Constant offsets preserve the recorded speeds and accelerations; clocks,
FoV/radio radii, birth priors, truth generator and measurement model remain
the v1 values. This checks modest layout/contact variation, not independent
robot navigation tasks or localization error. Exact poses are still known.

## Frozen arms

`local`, `fov`, `lineage`, `mil`, `mil_support`, `recent`,
`lineage_recent`, `qualified_exist`, `confirmed_exist`.

All existing arms retain their definitions. `mil_support` is the shared-label
LMB specialization of Gao et al. (2020) label-subspace MIL: for each label,
form its set of represented sources, renormalize the scheduled weights on
that set, and call the existing LMB-constrained MIL formula. This is
equivalent to grouping labels by their source-membership pattern. It differs
from `mil`'s zero extension. Labels are already common; no assignment module
or unknown-label result is claimed. An untouched prior is represented and
therefore still participates. No truth or current FoV mask defines support.
Both MIL variants use the same eight-component output cap as the original
configuration. Retain this bound and diagnose truncation separately if used.

The v3 confirmation bit and local diagnostic arrays are carried by ALL
arms; this must not affect old-arm numeric outputs. All nonlocal arms have
identical actual serialization, 16384-byte packets, 128 B/node/frame modeled
control, one round, link loss uniforms, and attempted/delivered messages.
The raw payload can differ with component count; padding is not a bandwidth
improvement. Original sources and input caches remain untouched.

## Analysis contract

Independently recompute all node-frame OSPA/count/matched errors and GOSPA
components using exhaustive assignment. Count paired independent episodes,
not frames/robots, as replicates. Report all arms and all seeds. Main
comparisons: qualified_exist vs lineage; lineage_recent vs qualified_exist;
and confirmed_exist vs qualified_exist. Also compare FoV-aware and both MIL
controls. Show absolute errors, paired mean differences, 95% percentile
bootstrap intervals across 20 seeds (10,000 resamples, fixed seed 8301),
common-target RMSE with support, and no-new/departure false costs.
Intervals are descriptive and not multiplicity-adjusted; do not call them
confirmatory significance tests. Preserve the original dev gate status.

Use no validation outcomes to retune weights, confirmation, priors, output
rules, trajectories, or exclusions. Any implementation failure is logged,
corrected, and rerun for every affected paired arm. A weak or adverse result
narrows paper claims rather than initiating another search.
