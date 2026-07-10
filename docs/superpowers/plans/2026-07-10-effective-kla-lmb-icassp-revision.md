# Effective KLA-LMB ICASSP Revision Implementation Plan
> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rebuild the ICASSP paper around a scoped fusion-operator-sufficient moment message, make the implementation and communication accounting match that claim, collect fresh paired evidence, and deliver a reproducible 4+1-page manuscript.

**Architecture:** Factor the current receiver into one idempotent label-wise moment projection followed by regularized Gaussian canonical fusion. Use the same projection at the sender, transport both full and projected posteriors through one versioned application-layer binary codec, and count encoded bytes before and after delivery. Validate the operator identity with property tests, then run a frozen two-arm static-topology experiment whose structured artifacts are the only source for paper figures and numbers.

**Tech Stack:** MATLAB/Octave, MATLAB structs and `uint8` binary serialization, existing GA-LMB simulator, Python 3 with matplotlib for figures, LaTeX/ICASSP `spconf`, Tectonic, Git.

---

## Scope and frozen design decisions

- The paper studies the implemented **single-round projected Gaussian KLA-LMB receiver**, not exact Gaussian-mixture KLA/GCI in general.
- `fusion-sufficient` means output-equivalent for the specified receiver; it does not mean the posterior densities are identical or statistically sufficient in the Fisher--Neyman sense.
- Main-paper experiments contain exactly two arms: static periodic full-GM message and static periodic projected-moment message.
- Both arms use identical topology, schedule, active-label threshold, spatial/existence weights, measurements, trajectories, and pre-generated delivery uniforms.
- Main-paper configurations set `lightCovarianceInflationEnabled=false`, `modeAwareFusionWeights=false`, and all dynamic-topology options off.
- The fresh confirmatory batch is pre-registered as paired seeds 82--131 (`baseSeed=81`, 50 trials); the exact implementation commit is frozen after the disjoint N5 smoke passes.
- The byte metric is encoded application-layer message length. It excludes MAC/PHY framing, IP/transport headers, MTU fragmentation, retransmission, and size-dependent loss.
- The previous seeds 32--81 report remains development/selection evidence and is not called held-out or confirmatory.

## Task 0: Commit the reviewed strategy checkpoint

**Files:**

- Add: `docs/icassp2027_paper/REVISION_GUIDE_CN.md`
- Force-add because the directory is ignored: `docs/superpowers/plans/2026-07-10-effective-kla-lmb-icassp-revision.md`

- [ ] Verify the guide contains exactly three reviewer reports and one cross-review synthesis, and that both documents pass whitespace checks.

- [ ] Stage only these two files; never use `git add .` in this dirty generated-artifact worktree:

```bash
git add docs/icassp2027_paper/REVISION_GUIDE_CN.md
git add -f docs/superpowers/plans/2026-07-10-effective-kla-lmb-icassp-revision.md
git diff --cached --check
git diff --cached --stat
```

- [ ] Commit with an English title and bilingual body, then push:

```bash
git commit -m "Document ICASSP revision strategy" -m "English:
- Record the adversarial review, claim boundaries, hard gates, and chosen fusion-sufficient story.
- Add a test-first implementation and evidence plan with scoped checkpoints.

中文：
- 固化对抗审查、claim 边界、硬 gate 与 fusion-sufficient 主线。
- 增加测试优先的实现和证据计划，并明确 scoped checkpoints。"
git push
```

## Task 1: Establish one idempotent label-wise moment projection

**Files:**

- Create: `multisensorLmb/projectLmbObjectMoments.m`
- Create: `multisensorLmb/regularizeCovarianceForSolve.m`
- Modify: `multisensorLmb/compressLmbPosterior.m`
- Modify: `multisensorLmb/fuseLmbPosteriorsByLabel.m`
- Create: `tests/test_lmb_moment_projection.m`
- Modify: `tests/test_dual_threshold_event_trigger.m`

- [ ] Write the failing projection test first with `rng(20270710)`. Cover one component, non-normalized positive weights, zero/non-finite weights, a random 1--20-component mixture, and a poorly conditioned covariance such as `diag([1e20, 1, 1, 1])`.

The central oracle is idempotence, not only agreement with a hand-written mean:

```matlab
[mu1, Sigma1] = projectLmbObjectMoments(object, model.xDimension);
projected = object;
projected.numberOfGmComponents = 1;
projected.w = 1;
projected.mu = {mu1};
projected.Sigma = {Sigma1};
[mu2, Sigma2] = projectLmbObjectMoments(projected, model.xDimension);
assert(isequaln(mu1, mu2));
assert(isequaln(Sigma1, Sigma2));
```

For the poorly conditioned case, additionally compare each element rather than scaling one tolerance by the largest eigenvalue, and assert that the small-eigenvalue directions receive no second jitter. A global tolerance based on `norm(Sigma,inf)` is forbidden because it would hide the reproduced `1e-9` drift next to a `1e20` entry.

- [ ] Run the new test and confirm it fails because the shared helper does not exist:

```bash
octave --quiet --eval "setPath; addpath('tests'); test_lmb_moment_projection;"
```

Expected result: Octave reports `projectLmbObjectMoments` as undefined.

- [ ] Implement `projectLmbObjectMoments`. Normalize finite nonnegative weights; use uniform weights when their sum is zero. Canonicalize every component covariance as `(Sigma_m + Sigma_m') / 2` **before** it enters the weighted mixture-covariance sum, matching the codec's component-level canonicalization order; finish the accumulated result with one final symmetry assignment. Do **not** add `rcond` jitter inside this projection.

- [ ] Replace the private moment-matching code in both `compressLmbPosterior` and `fuseLmbPosteriorsByLabel` with the shared helper. Keep covariance inflation after projection only for legacy non-paper configurations.

- [ ] Implement `regularizeCovarianceForSolve` with a fixed Cholesky-first policy: symmetrize and return an already-SPD matrix unchanged; only when Cholesky fails, try diagonal jitter `1e-12 * max(1,max(abs(diag(Sigma))))`, multiplying by 10 for at most 16 attempts, then raise an error. This replaces the current `rcond`-triggered unconditional jitter on poorly conditioned but SPD matrices.

- [ ] In `fuseLmbPosteriorsByLabel`, regularize each projected covariance exactly once immediately before a solve. Replace `inv(A)` with `A \ eye(size(A))`. Rework `logDet` to consume that already-regularized SPD copy without invoking the regularizer again, and reuse the same copy for the precision solve and log determinant. Apply the same rule to the fused covariance. Keep the solve regularizer separate from the projection so `P(P(x))=P(x)` remains true.

- [ ] Add a direct full-versus-projected fusion test over random labels and weights. Include a multi-component input whose covariances are slightly asymmetric to catch component-level canonicalization-order drift. Compare sorted label sets and require `isequaln` for every per-label `r`, `mu`, and `Sigma`. Because both paths must reach the same projected binary64 inputs before the same canonical solve, a nonzero residual is a defect, not a tolerance choice.

- [ ] Run focused and legacy tests:

```bash
octave --quiet --eval "setPath; addpath('tests'); test_lmb_moment_projection; test_dual_threshold_event_trigger;"
```

Expected result: both tests print their pass messages and Octave exits 0.

- [ ] Run `git diff --check`, review only the six listed scoped implementation/test files, then stage them explicitly:

```bash
git add multisensorLmb/projectLmbObjectMoments.m multisensorLmb/regularizeCovarianceForSolve.m
git add multisensorLmb/compressLmbPosterior.m multisensorLmb/fuseLmbPosteriorsByLabel.m
git add tests/test_lmb_moment_projection.m tests/test_dual_threshold_event_trigger.m
git diff --cached --check
```

- [ ] Commit and push:

```bash
git commit -m "Unify projected LMB moment computation" -m "English: Make sender and receiver use one idempotent label-wise moment projection; separate projection from numerical regularization and add boundary/property tests." -m "中文：统一发送端与接收端的逐标签矩投影并保证幂等；将投影与数值正则化分离，并补充边界与属性测试。"
git push
```

## Task 2: Implement the versioned application-layer wire codec

**Files:**

- Create: `multisensorLmb/getLmbWireSchema.m`
- Create: `multisensorLmb/encodeLmbWireMessage.m`
- Create: `multisensorLmb/decodeLmbWireMessage.m`
- Create: `tests/test_lmb_wire_codec.m`

The v1 layout is little-endian and shared by full and projected messages:

| Region | Fields | Bytes |
|---|---|---:|
| Header | magic `LMBW`; version `u8`; event type `u8`; state dimension `u8`; flags `u8=3`; sender/receiver `u16`; time/object count/total bytes `u32` | 24 |
| Object header | birth time/location `u32`; existence `binary64`; component count/reserved `u16` | 20 |
| Gaussian component | weight, `d` mean values, `d(d+1)/2` upper-triangle covariance values, all `binary64` | `8[1+d+d(d+1)/2]` |

For objects `l=1,...,L`, the exact length is

```text
24 + sum_l {20 + M_l * 8 * [1 + d + d(d+1)/2]} bytes.
```

- [ ] Write the failing codec tests before implementation. Include full, projected, mixed, and empty-object messages; metadata round trip; exact length formula; decoded `fieldnames` equality with `model.object`; and two independent little-endian golden fixtures: binary64 `1.0` is `00 00 00 00 00 00 f0 3f`, while a complete 24-byte header uses sender `0x0102 -> 02 01`, receiver `0x0304 -> 04 03`, and time `0x01020304 -> 04 03 02 01` at their exact schema offsets. Fix `flags=uint8(3)`: bit 0 is packed upper triangle, bit 1 is IEEE-754 binary64, and all other bits are zero.

- [ ] Add malformed-input tests for bad magic/version/flags/dimension, light messages with more than one component, zero components, inconsistent `w/mu/Sigma` lengths, negative weights, non-integer or out-of-range labels, invalid `r`, NaN/Inf, truncated bytes, extra bytes, and `totalBytes` mismatch. Validate before casting that `d<=255`, sender/receiver fit `uint16`, and time/object count/total bytes fit `uint32`. Check remaining length before allocating decoded arrays.

- [ ] Run the codec test and confirm the expected undefined-function failure:

```bash
octave --quiet --eval "setPath; addpath('tests'); test_lmb_wire_codec;"
```

- [ ] Implement `getLmbWireSchema` as the single source for magic, version, flags, widths, and fixed byte counts.

- [ ] Implement `encodeLmbWireMessage(objects, metadata, model)`. Return a row `uint8` vector and stats whose `encodedBytes` is exactly `numel(bytes)`. Filter/select active objects before calling the encoder; the encoder itself validates every object and encodes empty messages as the 24-byte header.

- [ ] Pack covariance only after checking shape, finiteness, and approximate symmetry; encode `(Sigma+Sigma')/2` in the order `Sigma(triu(true(d)))`. Do not quantize to single precision.

- [ ] Implement `decodeLmbWireMessage(bytes, model)`. Reconstruct covariance from the same upper-triangle mask and mirror only the strict triangle. Instantiate every decoded object from the `model.object` template, fill only the transported posterior fields, and initialize `trajectoryLength=0`, `trajectory=zeros(d,0)`, `timestamps=zeros(1,0)`.

- [ ] Make byte packing portable between MATLAB and Octave. `typecast` is native-endian, so detect the host using `typecast(uint16(1),'uint8')` and swap 2/4/8-byte values when necessary. Force all encoded chunks to row `uint8` arrays.

- [ ] Run codec and projection tests:

```bash
octave --quiet --eval "setPath; addpath('tests'); test_lmb_wire_codec; test_lmb_moment_projection;"
```

Expected result: labels, `r`, weights, means, and exactly symmetric covariances round-trip bit-exactly; an accepted approximately symmetric covariance decodes exactly to `(Sigma+Sigma')/2`; malformed cases raise the expected errors; Octave exits 0.

- [ ] Run `git diff --check`, inspect the schema and both directions together, then stage explicitly:

```bash
git add multisensorLmb/getLmbWireSchema.m multisensorLmb/encodeLmbWireMessage.m
git add multisensorLmb/decodeLmbWireMessage.m tests/test_lmb_wire_codec.m
git diff --cached --check
```

- [ ] Commit and push:

```text
Add typed LMB application message codec

English:
- Define a versioned binary schema shared by full and projected LMB messages.
- Add strict encode/decode validation and portable round-trip tests.

中文：
- 定义 full 与 projected LMB 消息共用的版本化二进制 schema。
- 增加严格的编解码校验与跨 MATLAB/Octave 往返测试。
```

```bash
git commit -m "Add typed LMB application message codec" -m "English: Define a versioned binary schema shared by full and projected LMB messages; add strict validation and portable round-trip tests." -m "中文：定义 full 与 projected LMB 共用的版本化二进制 schema；增加严格校验和跨 MATLAB/Octave 往返测试。"
git push
```

## Task 3: Route the simulator through the codec and split attempted/delivered bytes

**Files:**

- Modify: `multisensorLmb/runEventTriggeredDistributedLmbFilter.m`
- Modify: `multisensorLmb/estimateLmbPayloadSize.m`
- Modify: `tests/test_dual_threshold_event_trigger.m`
- Modify: `RUN/GA/runMultisensorFilters_formation_4plus4_DualThresholdEventTriggerCompare.m`

- [ ] First add failing accounting assertions for three deterministic cases:

```matlab
% All drops: an encoded attempt exists but nothing is delivered.
assert(summary.attemptedPayloadBytes > 0);
assert(summary.deliveredPayloadBytes == 0);

% Forced delivery: both byte totals are equal and the old alias is delivered.
assert(summary.attemptedPayloadBytes == summary.deliveredPayloadBytes);
assert(summary.payloadBytes == summary.deliveredPayloadBytes);

% In every case.
assert(summary.deliveredPayloadBytes <= summary.attemptedPayloadBytes);
```

Also test an event with zero active objects: it still attempts a 24-byte header.

- [ ] Run `test_dual_threshold_event_trigger` and confirm the new accounting fields are missing.

- [ ] Move payload construction into the `eventType>0` block before `simulateDelivery`. Build once, encode once, and store `attemptedPayloadBytes=numel(bytes)` before the delivery decision.

- [ ] On successful delivery, decode those exact bytes and place the decoded objects—not the sender's in-memory struct—into `currentMessages` and `receivedCache`. Assert decoded sender, receiver, time index, and event type equal the current edge/time/event before routing the message, then record `deliveredPayloadBytes`.

- [ ] Initialize edge-time arrays for attempted and delivered bytes and propagate their sums, plus `payloadDeliveryRatio`, into `diagnostics.summary`. Define the ratio as 0 when attempted bytes are 0; otherwise use exactly delivered/attempted. Add an `eventPolicy='none'` no-attempt test. Keep `summary.payloadBytes` only as a delivered-byte compatibility alias.

- [ ] Remove `estimateLmbPayloadSize` from the production communication path. Retain it as an explicitly documented legacy scalar-equivalent diagnostic only if old experiments/tests still call it; no paper field named `*Bytes` may derive from it.

- [ ] Extend the generic comparison runner with per-trial `attemptedPayloadBytes` and `deliveredPayloadBytes` arrays. Preserve old `payloadBytes` output for historical reports, but make new reports label it as the compatibility delivered alias.

- [ ] Verify that encoding is pure and does not consume random numbers, so pre-generated `commConfig.linkUniforms` still makes delivery masks identical between arms.

- [ ] Run the accounting, codec, projection, and existing trigger tests:

```bash
octave --quiet --eval "setPath; addpath('tests'); test_lmb_wire_codec; test_lmb_moment_projection; test_dual_threshold_event_trigger;"
```

- [ ] Run a one-trial full/light smoke through the existing comparison driver and inspect `delivered <= attempted` for both arms:

```bash
octave --quiet --eval "setPath; addpath('RUN/GA'); [~,s]=runMultisensorFilters_formation_4plus4_DualThresholdEventTriggerCompare(1,1000,true,false,'default',{'Periodic full posterior','Periodic light posterior on static topology'},struct('includeFinalPeriodicLightVariants',true,'simulationLength',20)); disp(s.communication);"
```

- [ ] Run `git diff --check`, then stage only the accounting path:

```bash
git add multisensorLmb/runEventTriggeredDistributedLmbFilter.m multisensorLmb/estimateLmbPayloadSize.m
git add tests/test_dual_threshold_event_trigger.m RUN/GA/runMultisensorFilters_formation_4plus4_DualThresholdEventTriggerCompare.m
git diff --cached --check
```

- [ ] Commit and push:

```text
Measure encoded attempted and delivered LMB traffic

English:
- Route successful posterior messages through the typed codec.
- Separate attempted and delivered application-layer bytes while retaining a compatibility alias.

中文：
- 让成功投递的后验消息实际经过 typed codec。
- 拆分 attempted 与 delivered 应用层字节，并保留兼容字段。
```

```bash
git commit -m "Measure encoded attempted and delivered LMB traffic" -m "English: Route successful messages through the typed codec and split attempted from delivered application-layer bytes." -m "中文：让成功消息实际经过 typed codec，并拆分 attempted 与 delivered 应用层字节。"
git push
```

## Task 4: Add fusion-output equivalence instrumentation

**Files:**

- Create: `multisensorLmb/snapshotLmbPosterior.m`
- Create: `multisensorLmb/compareLmbPosteriorSnapshots.m`
- Modify: `multisensorLmb/runEventTriggeredDistributedLmbFilter.m`
- Create: `tests/test_lmb_posterior_equivalence.m`

- [ ] Write a failing test that compares two snapshot sequences with permuted labels, then perturbs existence, one mean coordinate, one covariance entry, and one label. The comparator must report comparison count, label-set mismatch count, max `|delta r|`, max `||delta mu||_inf`, and max `||delta Sigma||_inf`.

- [ ] Implement compact snapshots containing only sorted labels, existence probabilities, means, and covariances. Never capture trajectories or full GM component histories.

- [ ] Add an opt-in `capturePosteriorSnapshots` trigger setting. When true, record the compact fused posterior after each receiver/time update; when false, retain no large cell payload. The paper runner in Task 5 must assert this setting is true.

- [ ] Implement the comparator with explicit missing-label reporting. Do not silently compare only the intersection.

- [ ] Add an end-to-end property test: encode/decode full inputs, fuse them; project then encode/decode the same inputs, fuse them; compare every label's output. Include missing labels across sources and a near-singular valid covariance.

- [ ] Run:

```bash
octave --quiet --eval "setPath; addpath('tests'); test_lmb_posterior_equivalence; test_lmb_wire_codec; test_lmb_moment_projection;"
```

- [ ] Stage the four instrumentation files explicitly:

```bash
git add multisensorLmb/snapshotLmbPosterior.m multisensorLmb/compareLmbPosteriorSnapshots.m
git add multisensorLmb/runEventTriggeredDistributedLmbFilter.m tests/test_lmb_posterior_equivalence.m
git diff --cached --check
```

- [ ] Commit and push the instrumentation separately so confirmatory artifacts can cite a stable implementation commit:

```text
Add projected fusion equivalence diagnostics

English:
- Capture compact posterior snapshots and compare full/projected outputs per label.
- Add end-to-end codec and projected-fusion equivalence tests.

中文：
- 记录紧凑后验快照并逐标签比较 full/projected 输出。
- 增加 codec 到 projected fusion 的端到端等价测试。
```

```bash
git commit -m "Add projected fusion equivalence diagnostics" -m "English: Capture compact posterior snapshots and compare full/projected outputs per label; add end-to-end codec equivalence tests." -m "中文：记录紧凑后验快照并逐标签比较 full/projected 输出；增加 codec 端到端等价测试。"
git push
```

## Task 5: Freeze a paper-specific two-arm runner and artifact contract

**Files:**

- Create: `RUN/GA/runFusionSufficientMomentExchangeConfirmatory.m`
- Create: `RUN/GA/writeFusionSufficientEvidence.m`
- Create: `RUN/GA/validateFusionSufficientEvidence.m`
- Modify: `RUN/GA/runMultisensorFilters_formation_4plus4_DualThresholdEventTriggerCompare.m`
- Create: `tests/test_icassp_moment_exchange_runner.m`
- Create: `tests/test_icassp_moment_exchange_evidence.m`
- Create: `docs/icassp2027_paper/EXPERIMENT_PROTOCOL_CN.md`

- [ ] Write a dry-run test that obtains the config without launching simulation and asserts:

```matlab
assert(isequal(config.armSelection, { ...
    'Periodic full posterior', ...
    'Periodic light posterior on static topology'}));
assert(config.numberOfTrials == 50);
assert(config.baseSeed == 81);              % paired seeds 82:131
assert(~config.lightCovarianceInflationEnabled);
assert(~config.modeAwareFusionWeights);
assert(~config.includeDynamicTopologyVariants);
assert(config.capturePosteriorSnapshots);
assert(config.requiredMaxExistenceResidual == 0);
assert(config.requiredMaxMeanResidual == 0);
assert(config.requiredMaxCovarianceResidual == 0);
assert(config.bootstrapSeed == 20270710);
assert(config.bootstrapResamples == 10000);
```

- [ ] Implement the exact interface

```matlab
function [reportPath, csvPath, summaryPath, summary, config] = ...
    runFusionSufficientMomentExchangeConfirmatory( ...
        runExperiment, numberOfTrials, baseSeed)
```

with defaults `runExperiment=true`, `numberOfTrials=50`, and `baseSeed=81`. `runExperiment=false` returns the frozen default config without simulation. The only permitted smoke override is `(true,5,1000)`, which uses seeds 1001--1005 and `N5_SEEDS1001_1005` artifact names. The default console output must print commit hash, trial count, exact seed interval, arm names, simulation length, snapshot settings, residual gates, bootstrap settings, and byte semantics before any trial starts.

- [ ] Make the generic runner propagate attempted/delivered arrays, delivery masks, and optional snapshot comparisons. For the paper runner, assert full/light attempted masks and delivered masks are identical for each trial.

- [ ] Implement the evidence writer with atomic publication: write `.mat`, CSV, and Markdown to temporary sibling names, close and validate them, then rename to final names only after both the run and validation complete. Each completed run creates:

  - an ignored local `.mat` summary containing the complete simulator output;
  - the frozen-N50 CSV `RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.csv`, with seed, full/light attempted/delivered bytes, paired reductions, tracking/consensus metrics, and equivalence residuals;
  - the frozen-N50 report `RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.md`, with config, Git commit, schema version, aggregate statistics, min/median/max, paired percentile bootstrap interval, local `.mat` path/SHA-256, CSV SHA-256, and regeneration command. Compute hashes with the host command `shasum -a 256` and store the lowercase 64-hex digest.

Smoke runs use `N5_SEEDS1001_1005` in their filenames and must never overwrite the frozen-N50 names.

For each seed `s`, define `rho_s = 100 * (B_full,s - B_projected,s) / B_full,s` from attempted application-layer bytes. The primary reduction is `mean(rho_s)` across the 50 paired seeds. Use RNG seed `20270710`, draw exactly 10,000 bootstrap samples of 50 `rho_s` values with replacement, and report the 2.5/97.5 percentiles of those resampled means. Do not bootstrap aggregate byte ratios, and do not substitute BCa/basic intervals after results are observed.

- [ ] Implement `validateFusionSufficientEvidence(summaryPath,csvPath,reportPath)`. It reloads the ignored `.mat`, recomputes every CSV row and aggregate, verifies seeds/config/schema/commit, checks exact-zero residuals and matching masks, verifies SHA-256 entries, and raises an error on the first mismatch. `tests/test_icassp_moment_exchange_evidence.m` must corrupt one CSV cell and prove the validator rejects it.

- [ ] Document the experiment protocol before seeing confirmatory results. The document must state seeds 82--131, two arms, one changed variable, metric definitions, numerical tolerances, application-layer boundary, and the rule that any algorithm/config change invalidates the batch.

- [ ] Run dry configuration and tests:

```bash
octave --quiet --eval "setPath; addpath('RUN/GA'); addpath('tests'); test_icassp_moment_exchange_runner; test_icassp_moment_exchange_evidence; [~,~,~,~,c]=runFusionSufficientMomentExchangeConfirmatory(false); disp(c);"
```

- [ ] Run `git diff --check`, inspect the printed config, then stage exactly the protocol files:

```bash
git add RUN/GA/runFusionSufficientMomentExchangeConfirmatory.m RUN/GA/writeFusionSufficientEvidence.m
git add RUN/GA/validateFusionSufficientEvidence.m RUN/GA/runMultisensorFilters_formation_4plus4_DualThresholdEventTriggerCompare.m
git add tests/test_icassp_moment_exchange_runner.m tests/test_icassp_moment_exchange_evidence.m
git add docs/icassp2027_paper/EXPERIMENT_PROTOCOL_CN.md
git diff --cached --check
```

- [ ] Commit and push **before** any smoke or fresh N50:

```text
Freeze ICASSP moment-exchange validation protocol

English:
- Add the static full/projected two-arm runner and structured evidence contract.
- Pre-register seeds, byte semantics, equivalence tolerances, and artifact provenance.

中文：
- 新增 static full/projected 两臂 runner 与结构化证据协议。
- 预先固定 seeds、字节语义、等价容差和 artifact provenance。
```

```bash
git commit -m "Freeze ICASSP moment-exchange validation protocol" -m "English: Add the static full/projected runner and evidence validator; pre-register seeds, byte semantics, exact residual gates, and provenance." -m "中文：新增 static full/projected runner 与证据验证器；预先固定 seeds、字节语义、精确残差 gate 和 provenance。"
git push
```

## Task 6: Run smoke validation and fresh confirmatory N50

**Files:**

- Generate under: `RUN/GA/` with the frozen report/CSV names defined in Task 5 and a timestamped ignored raw `.mat`
- Commit: `RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.md` and `RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.csv`
- Keep local/ignored: `.mat` by the repository's `*.mat` rule and tee logs under ignored `.superpowers/logs/`; record the `.mat` path/SHA-256 in the report without claiming the raw artifact is archived in Git

- [ ] Create the log location without changing tracked files:

```bash
mkdir -p .superpowers/logs
```

- [ ] Run a paired N5 smoke on disjoint seeds 1001--1005 through the same runner path and keep a durable tee log:

```bash
set -o pipefail
octave --quiet --eval "setPath; addpath('RUN/GA'); [r,c,s,~,~]=runFusionSufficientMomentExchangeConfirmatory(true,5,1000); validateFusionSufficientEvidence(s,c,r);" 2>&1 | tee .superpowers/logs/fusion_sufficient_n5_seeds1001_1005.log
```

Expected result: two arms only; identical attempted/delivered masks; every trial has positive attempted-byte reduction; no label mismatch; residuals within the frozen tolerances; `.mat`, CSV, and Markdown artifacts all exist.

- [ ] Confirm the same Octave invocation above returned all three exact paths and completed `validateFusionSufficientEvidence(summaryPath,csvPath,reportPath)` before the shell returned success. Stop if any schema/config/provenance field is absent or any equivalence assertion fails. Do not loosen tolerances after observing results.

- [ ] Launch the frozen N50 with a durable tee log:

```bash
set -o pipefail
octave --quiet --eval "setPath; addpath('RUN/GA'); [r,c,s,~,~]=runFusionSufficientMomentExchangeConfirmatory(true); validateFusionSufficientEvidence(s,c,r);" 2>&1 | tee .superpowers/logs/fusion_sufficient_n50_seeds82_131.log
```

Handoff/monitor command:

```bash
tail -f .superpowers/logs/fusion_sufficient_n50_seeds82_131.log
```

- [ ] On completion, require all of the following before using the evidence:

  - exactly 50 paired seeds 82--131;
  - full/light attempted and delivered masks identical within each seed;
  - zero label-set mismatches;
  - frozen `r/mu/Sigma` tolerances satisfied;
  - `deliveredPayloadBytes <= attemptedPayloadBytes` in every arm/trial;
  - projected attempted-byte reduction positive in every trial;
  - report/CSV values reproduce directly from the `.mat` summary.

- [ ] If a gate fails after the 82--131 batch starts, stop Tasks 6--9 and downgrade that batch to development evidence. Fix code/config, then revise this plan, `EXPERIMENT_PROTOCOL_CN.md`, runner defaults/allowed parameters, report/CSV names, staging paths, figure input, and clean-audit commands from `SEEDS82_131` to the next disjoint batch `SEEDS132_181` (`baseSeed=131`). Commit and push that complete protocol amendment before running. Never rerun 82--131 as confirmatory or let a later task read its stale artifact.

- [ ] If all gates pass, run the validator on the exact N50 paths, run `git diff --check`, and stage only the lightweight final report/CSV:

```bash
git add RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.md
git add RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.csv
git diff --cached --check
```

- [ ] Commit and push:

```text
Record confirmatory moment-exchange evidence

English:
- Add the frozen paired-seed full/projected application-byte results.
- Record fusion-output residuals, provenance, and artifact integrity checks.

中文：
- 记录冻结配对 seeds 的 full/projected 应用层字节结果。
- 保存融合输出残差、provenance 与 artifact 完整性检查。
```

```bash
git commit -m "Record confirmatory moment-exchange evidence" -m "English: Add the frozen paired-seed application-byte results with exact fusion-output residuals and provenance checks." -m "中文：记录冻结配对 seeds 的应用层字节结果，并保存精确融合输出残差与 provenance 检查。"
git push
```

## Task 7: Replace hard-coded figures with evidence-driven paper figures

**Files:**

- Rewrite: `docs/icassp2027_paper/scripts/generate_figures.py`
- Replace: `docs/icassp2027_paper/figures/payload_graph_schematic.pdf`
- Replace: `docs/icassp2027_paper/figures/payload_graph_schematic.png`
- Replace: `docs/icassp2027_paper/figures/heldout_tradeoff.pdf`
- Replace: `docs/icassp2027_paper/figures/heldout_tradeoff.png`
- Create: `docs/icassp2027_paper/figures/figure_manifest.json`
- Create: `tests/test_icassp2027_figures.py`

- [ ] Write a failing Python test that calls the figure script with the final CSV path and asserts the expected seed count is 50, no dynamic arm name appears, both PDF/PNG pairs are produced, and output files are nonempty. Require `figure_manifest.json` to contain the repository-relative evidence path, evidence SHA-256, row count, seed interval, SHA-256 of `generate_figures.py`, and SHA-256 for every generated PDF/PNG. Use source/blob hashes rather than `HEAD` so the manifest has no circular dependency on the commit that adds itself.

- [ ] Replace Figure 1 with a commutative two-path schematic:

```text
Full GM message -> receiver projection P -> projected KLA G -> fused output
      | sender projection P -> moment wire message -> projected KLA G -> same output
```

Show the commuting equality and assumptions; do not depict graph sparsification or a sequential three-panel pipeline.

- [ ] Replace Figure 2 with paired confirmatory evidence read from CSV: one panel for per-seed attempted-byte reduction/distribution and one panel for fusion-output residuals or paired tracking deltas. Remove the four-arm aggregate scatter and all hard-coded paper numbers.

- [ ] Use final-size fonts of at least about 7 pt, colorblind-safe colors, distinct line/marker styles, and black-and-white distinguishability. Keep vector PDF as the LaTeX source and PNG only for review.

- [ ] Make generation byte-deterministic: sort manifest keys, omit wall-clock timestamps, set stable PDF metadata, and use `SOURCE_DATE_EPOCH=0`. The clean-worktree regeneration gate in Task 9 must leave no diff.

- [ ] Run:

```bash
/Users/dex/miniconda3/bin/python3 -m pytest -q tests/test_icassp2027_figures.py
SOURCE_DATE_EPOCH=0 /Users/dex/miniconda3/bin/python3 docs/icassp2027_paper/scripts/generate_figures.py --evidence RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.csv
```

- [ ] Visually inspect both PNGs at publication size and verify labels, arrows, legends, and residual scales.

- [ ] Stage only the figure source/test/manifest and four outputs:

```bash
git add docs/icassp2027_paper/scripts/generate_figures.py tests/test_icassp2027_figures.py
git add docs/icassp2027_paper/figures/figure_manifest.json
git add docs/icassp2027_paper/figures/payload_graph_schematic.pdf docs/icassp2027_paper/figures/payload_graph_schematic.png
git add docs/icassp2027_paper/figures/heldout_tradeoff.pdf docs/icassp2027_paper/figures/heldout_tradeoff.png
git diff --cached --check
```

- [ ] Commit and push figure sources/tests/outputs separately:

```text
Rebuild ICASSP figures from confirmatory evidence

English:
- Replace the topology story with an operator-commuting schematic.
- Generate paired communication and equivalence evidence directly from the frozen CSV.

中文：
- 用算子交换图替换原 topology 叙事图。
- 直接从冻结 CSV 生成配对通信与等价性证据图。
```

```bash
git commit -m "Rebuild ICASSP figures from confirmatory evidence" -m "English: Replace the topology story with an operator-commuting schematic and generate paired evidence directly from the frozen CSV." -m "中文：用算子交换图替换 topology 叙事，并直接从冻结 CSV 生成配对证据。"
git push
```

## Task 8: Rewrite the manuscript around the scoped operator claim

**Files:**

- Modify: `docs/icassp2027_paper/main.tex`
- Modify: `docs/icassp2027_paper/sections/01_introduction.tex`
- Modify: `docs/icassp2027_paper/sections/02_related_work.tex`
- Modify: `docs/icassp2027_paper/sections/03_method.tex`
- Modify: `docs/icassp2027_paper/sections/04_experiments.tex`
- Modify: `docs/icassp2027_paper/sections/05_conclusion.tex`
- Modify: `docs/icassp2027_paper/refs.bib`
- Modify: `docs/icassp2027_paper/storyline.md`
- Regenerate: `docs/icassp2027_paper/main.pdf`
- Create: `tests/check_icassp2027_pdf.py`

- [ ] Before drafting, load the required `nature-writing`/`ml-paper-writing` guidance for a methods-style ICASSP paper and use `nature-polishing`/`humanizer` only after the scientific claims are stable.

- [ ] Change the title to the frozen wording `Fusion-Sufficient Moment Exchange for Distributed Projected KLA-LMB Tracking`. If it wraps, fix layout without changing the scientific scope.

- [ ] Rewrite the abstract in five moves: problem; receiver factorization; sender-side projection and proposition; exact codec/paired evaluation; scoped boundary. Delete “safer than event-triggered graph sparsification.”

- [ ] Rewrite the introduction as `unused mixture detail -> operator-induced representation -> theorem/codec/evidence`. Keep exactly three contributions: factorization/equivalence proposition, typed message implementation, and confirmatory validation.

- [ ] Rebuild related work around three categories: schedule/event-trigger reduction, component selection/partial consensus, and state/covariance or posterior projection. Verify publisher metadata and claims for at least these DOI records before adding BibTeX:

  - `10.1109/TSP.2022.3154227`
  - `10.1109/TCSII.2023.3238346`
  - `10.1016/j.sigpro.2023.109238`
  - `10.1016/j.sigpro.2025.110149`
  - `10.3390/electronics15020458`
  - `10.1109/TAES.2018.2882960`

- [ ] In Method, define `P`, write the actual receiver as `F_omega=G_omega circ P`, state the conditional proposition `F({pi_j})=F({P pi_j})`, and give a short proof from shared labels/weights and projection idempotence. State all failure conditions next to the proposition.

- [ ] Add the 24-byte schema and analytical payload formula. Call the measured quantity `attempted/delivered application-layer bytes`; do not use an unqualified `wire bytes` claim.

- [ ] Rewrite Experiments around the frozen two-arm batch. Report the exact final artifact-derived reduction, intervals, residuals, and tracking sanity checks. Remove dynamic arms, graph-connectivity proxy, arbitrary pass counts, and the old held-out label.

- [ ] Rewrite Conclusion in three sentences: operator result, measured application-byte result, and limitations. Do not reintroduce topology claims.

- [ ] Run a prohibited-claim scan:

```bash
rg -n -i "safer than|held[- ]out|lossless|dynamic topology|lambda_2|algebraic connectivity|general GM|wire bytes" docs/icassp2027_paper/main.tex docs/icassp2027_paper/sections
```

Expected result: no prohibited claims; any occurrence in a limitation is manually verified as explicitly negated/scoped.

- [ ] Generate figures from the exact evidence CSV. Build from the paper directory and save a review log:

```bash
cd docs/icassp2027_paper
set -o pipefail
SOURCE_DATE_EPOCH=0 tectonic main.tex 2>&1 | tee /tmp/icassp2027-tectonic.log
SOURCE_DATE_EPOCH=0 tectonic main.tex 2>&1 | tee /tmp/icassp2027-tectonic-second.log
```

Return to the repository root, then scan both logs:

```bash
rg -n "Overfull|undefined citations|undefined references|Citation.*undefined|Reference.*undefined|error:" /tmp/icassp2027-tectonic.log /tmp/icassp2027-tectonic-second.log
```

Expected result: the scan prints nothing and `main.pdf` exists.

- [ ] Implement standalone `tests/check_icassp2027_pdf.py` using `pypdf` for page/text checks and `pypdfium2` for rendering. It must assert exactly five pages; reject Introduction/Method/Experiments/Conclusion text on page 5; and render all pages to ignored `tmp/icassp2027_render/page-01.png` through `page-05.png`.

- [ ] Run the PDF test, then inspect all five PNGs with the image viewer for column balance, equations, captions, reference breaks, clipping, overlaps, and minimum figure font:

```bash
/Users/dex/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 tests/check_icassp2027_pdf.py
```

- [ ] Stage only the manuscript, bibliography, storyline, PDF test, and synchronized `main.pdf`:

```bash
git add docs/icassp2027_paper/main.tex docs/icassp2027_paper/main.pdf docs/icassp2027_paper/refs.bib docs/icassp2027_paper/storyline.md
git add docs/icassp2027_paper/sections/01_introduction.tex docs/icassp2027_paper/sections/02_related_work.tex
git add docs/icassp2027_paper/sections/03_method.tex docs/icassp2027_paper/sections/04_experiments.tex docs/icassp2027_paper/sections/05_conclusion.tex
git add tests/check_icassp2027_pdf.py
git diff --cached --check
```

- [ ] Commit and push the manuscript and synchronized `main.pdf`:

```text
Rewrite ICASSP paper around fusion-sufficient messages

English:
- Reframe the contribution as an operator-induced projected LMB message with explicit boundaries.
- Replace confounded topology evidence with codec-backed paired results and verified related work.

中文：
- 将贡献重构为带明确边界的算子诱导 projected LMB 消息。
- 用 codec 支撑的配对结果和核验后的相关工作替换混杂 topology 证据。
```

```bash
git commit -m "Rewrite ICASSP paper around fusion-sufficient messages" -m "English: Reframe the contribution as an operator-induced projected LMB message and replace confounded topology evidence with codec-backed paired results." -m "中文：将贡献重构为算子诱导的 projected LMB 消息，并以 codec 支撑的配对结果替换混杂 topology 证据。"
git push
```

## Task 9: Final clean-checkout and claim audit

**Files:**

- Modify only if an audit fails: implementation, tests, experiment report, paper source, generated figures, or `docs/icassp2027_paper/main.pdf`

- [ ] Run all focused MATLAB/Octave tests:

```bash
octave --quiet --eval "setPath; addpath('RUN/GA'); addpath('tests'); test_lmb_moment_projection; test_lmb_wire_codec; test_lmb_posterior_equivalence; test_dual_threshold_event_trigger; test_icassp_moment_exchange_runner; test_icassp_moment_exchange_evidence;"
```

- [ ] Run Python figure tests:

```bash
/Users/dex/miniconda3/bin/python3 -m pytest -q tests/test_icassp2027_figures.py
/Users/dex/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 tests/check_icassp2027_pdf.py
```

- [ ] Rebuild figures from the exact final CSV, rebuild LaTeX twice, rerun the PDF test, and use `git diff --check` while the intentional final changes are still unstaged. Do not use `git diff --exit-code` until the clean detached audit below.

- [ ] Audit every numeric claim in abstract, experiments, conclusion, table, and captions against the final CSV/Markdown report. Audit every theoretical claim against the proposition assumptions and the property tests.

- [ ] After Task 8 is committed and pushed, create a detached clean worktree at that exact commit:

```bash
git worktree add --detach /tmp/effective-kla-lmb-final-audit HEAD
```

From `/tmp/effective-kla-lmb-final-audit`, run the complete Octave command above, both Python tests, the exact evidence-driven figure command, and two Tectonic builds from `docs/icassp2027_paper`. Then run:

```bash
git diff --exit-code
git status --short
```

Expected result: both commands print nothing/exit 0 after deterministic regeneration. Return to the primary worktree and remove the clean audit worktree:

```bash
git worktree remove /tmp/effective-kla-lmb-final-audit
```

If any audit fails, remove the temporary worktree, fix in the primary worktree, make a new scoped commit/push, and repeat this detached audit at the new `HEAD`.

- [ ] Run final repository checks:

```bash
git diff --check
git status --short
```

Expected result: only the intended final files are staged; no raw temporary render pages, logs, unrelated user edits, or stale generated files enter the commit.

- [ ] If the audit required changes, list them with `git diff --name-only`, stage those paths explicitly (never `git add .`), rerun `git diff --cached --check`, and commit/push:

```bash
git commit -m "Finalize ICASSP reproducibility audit" -m "English: Resolve the final clean-checkout, artifact, and PDF audit findings." -m "中文：修复最终 clean-checkout、artifact 与 PDF 审计发现。"
git push
```

If no changes were required, do not manufacture an empty commit.

## Plan self-review checklist

- [ ] Every retained paper claim maps to a proposition, focused test, codec definition, or final evidence artifact.
- [ ] Full/projected comparisons change exactly one variable.
- [ ] No dynamic arm, event-trigger superiority claim, or symmetrized connectivity proxy remains in the main evidence.
- [ ] Byte fields come only from encoded `uint8` length; the old scalar estimator is visibly legacy.
- [ ] Projection and solve regularization are separate, and projection idempotence is tested.
- [ ] The codec is exercised on the actual receiver path, not only for accounting.
- [ ] Seeds, tolerances, schema, artifact format, and bootstrap RNG are frozen before N50.
- [ ] Figure/table numbers are data-driven and traceable to one exact CSV.
- [ ] The paper states the application-layer and size-independent-loss limitations.
- [ ] The final PDF satisfies ICASSP 4+1 pages and page 5 contains references only.
- [ ] No unresolved work marker, placeholder result, or unresolved citation remains.
- [ ] Each key checkpoint is committed narrowly and pushed before the next irreversible/long-running step.
