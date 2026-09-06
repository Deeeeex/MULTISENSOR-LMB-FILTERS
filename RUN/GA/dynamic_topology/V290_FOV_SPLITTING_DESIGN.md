# V290: FoV-aware local updating under fixed sparse routing

## Question

Can resolving the local predicted density at FoV boundaries improve joint
set recovery/localization under the unchanged V242 KLA route? V289 found
material pD discrepancies, but did not establish any tracking benefit.

## Scope

One default-off local-update change. Apply LeGrand--Ferrari Algorithm 1 to
each predicted Bernoulli GM before a scheduled measurement update. Use the
published R=4, lambda=0.001 split library, full-state direction rule (32),
and minimum conditional component weight 0.01. The fixed grid has 15 points
per dimension inside a Mahalanobis-radius-3 disk; these grid values are our
numerical choice, not claimed to reproduce the authors' demonstration setup.
Stop recursion by the published weight/inclusion criteria, not an arbitrary
two-level cap. Keep every child until the existing posterior reduction.

Labels, existence, total conditional mass and the first moment are unchanged
by splitting. The approximate split library does NOT preserve covariance
exactly. Position--velocity dependence is represented through full-state
covariance eigenvectors. The subsequent original LMB association update
changes both existence and spatial density; it is not a scalar-pD patch.
State-dependent pD and measurement noise are evaluated at refined component
means. Smooth within-FoV variation remains approximate.

Generate the complete original 160-step scene, then crop to 2 steps for
integration and 40 for the development screen; X36, seed 1301. Set the split
option only on the runtime model after measurement generation. Keep 120-degree
FoV, 300 m range, trajectories, observations, delivery uniforms, common labels,
filter RNG, Metropolis KLA, absent-label rule and posterior caps unchanged.
No MIL, prior exclusion, matching, new routing, auxiliary messages or sweep.

The first integration exposed the existing quadratic exact-copy grouping
cost on expanded GMs. A scoped implementation optimization sorts exact
finite-double parameter rows, while retaining first representatives and
original per-group summation order. It performs no rounding or approximate
merging. The original pairwise method remains the default; only this runner
selects sorted grouping, and unusual parameter representations fall back to
the original equality checks. Weight pruning and posterior caps are unchanged.

Reuse the saved V282 filter reference. Reuse its already-computed reference
metrics from the matching V288 result artifact; no MIL candidate metric is
used. Only the V290 candidate is filtered and scored.

## Risk Tier

L2 development experiment, self-check only. Neither this screen nor its
formula check establishes held-out validation or publication readiness.

## Claims

- C1: Splitting is a published local-density approximation, not new routing
  or an exact Bayes implementation (E1).
- C2: The intended comparison changes local update resolution only; packet
  content and bytes may change as posterior histories change (E2).
- C3: The explicit sorted-grouping option matches the legacy output and
  diagnostics on the focused duplicate/reduction fixture (E4); this is not
  independent validation of the complete filter.

## Evidence Ledger

- E1: [LeGrand and Ferrari author PDF](https://keithlegrand.com/wp/wp-content/uploads/2023/05/LeGrand-2022-Split-Happens-Imprecise-and-Negative-Information-in-Gaussian-Mixture-Random-Finite-Set-Filtering.pdf),
  JAIF 17(2):78--96 (2022), Table I, Section IV-D--G, equations (20)--(32),
  Algorithm 1. The author PDF was downloaded after web extraction timed out;
  the formula/algorithm page was rendered and read directly.
- E2: `multisensorLmb/splitLmbPredictionAtFovBoundary.m`, the default-off
  call in `updateLmbWithSensorMeasurement.m`, and the V290 runner define
  the local change and retain the ordinary downstream filter.
- E3: `/opt/homebrew/bin/octave --quiet --eval "addpath(genpath(pwd)); checkFovGaussianSplittingV290();"`
  exited 0: `V290 formula self-check PASS`. Analytic half-plane pD was
  0.800000000 under the old mean rule, 0.465593884 after splitting, and
  0.463407768 exactly. The fixture produced 142 components at depth 5;
  relative covariance change was 0.049773, not zero.
- E4: `/opt/homebrew/bin/octave --quiet --eval "addpath(genpath(pwd)); checkSortedGmGroupingV290();"`
  exited 0: exact output/diagnostic equality with duplicate, zero/tiny and
  tied weights, with/without reduction. The 300-component fixture took
  0.3311 s pairwise and 0.0203 s sorted (16.32 ratio), not an end-to-end
  speedup claim.
- E5: The initial two-step integration (session 84866, PID 22935) reached
  `Filter starting step 1/2` at 08:31:10 local time, 2026-09-06. A later
  `ps -p 22935 -o pid,etime,time,%cpu,state` showed elapsed 09:54, CPU time
  09:52.41 and 100% CPU. Native sampling exposed interpreter work but did
  not identify an M-file hotspot. Code inspection established the expanded
  posterior's pairwise exact-copy loop; E4 verified its scoped replacement.
  The still-live integration was deliberately interrupted with
  `kill -INT 22935` before editing runtime code; session 84866 exited 1.
  No raw tracking result was produced or scored. This was a computational
  implementation correction, not a restart after observation expiry or a
  response to unfavorable performance metrics.
- E6: Four scoped Lark paragraph replacements succeeded; the final
  `lark-cli docs +fetch --doc HcFFdtKIRovhHLxKrx5jVpiBpJh --scope keyword --keyword '目前卡点|局部更新、融合|下一道决策|这里的零' --detail with-ids --as user`
  returned revision 1267. The method paragraph separates local updating,
  fusion and routing and does not claim candidate tracking gains. The
  zero-pD paragraph now distinguishes component means from density support.
  No board or best-result table was modified.
- E7: The corrected two-step integration (session 16493) exited 0 and saved
  `evidence/tracking_aligned_v290/x36_prefix2_integration_seed1301/FOV_GAUSSIAN_SPLITTING_V290_REPORT.md`
  and the three metric CSVs. Filter time was 93.1 s, of which 8.8 s was
  splitting. Attempted/delivered route-mask differences were 0/0; all 72
  sensor-time RMSE cells were finite. E-OSPA was 139.697349 -> 141.828991,
  RMSE 11.831774 -> 12.015035, and attempted bytes 677584 -> 2921296
  (4.3113 times reference). This is a successful technical integration,
  not evidence of improved tracking or a completed 40-step screen. It ran
  with the uncommitted implementation over parent HEAD `7d94ceb`; the parent
  identifier in the raw artifact alone does not identify this implementation.
- E8: `/Users/dex/miniconda3/bin/python3 /Users/dex/.codex/skills/auto-research/scripts/evidence_lint.py RUN/GA/dynamic_topology/V290_FOV_SPLITTING_DESIGN.md`
  exited 0 with `PASS`; `git diff --check` also exited 0. These are document
  and whitespace checks, not independent scientific validation.

## Verification Record

Self-check only. The formula check covers mass/first moment, labels/existence,
positive covariance, correlation retention, a no-boundary bypass and an
analytic half-plane pD case. One additional exact-grouping check was required
by the integration's computational bottleneck. It checks identical outputs
and diagnostics, not tracking quality. No broad regression suite, hash audit
or independent reviewer has been run. The two-step integration completed;
freeze the unchanged runtime source before the single 40-step run.

## Risk and Escalation

Splitting can increase packet sizes and runtime; repeated approximation and
the existing posterior cap can discard density detail. A better pD integral
need not yield better tracking. The strict final M24/X36 joint objective is
unchanged. Failed or mixed candidates stay in experiment records.

## Reproducibility

Run from the isolated ICASSP worktree with Octave 11.1.0. The corrected
two-step integration is session 16493, using this command:

```sh
set -o pipefail
/opt/homebrew/bin/octave --no-gui --quiet --eval "addpath(genpath(pwd)); o=struct('baselineTracePath','RUN/GA/dynamic_topology/evidence/tracking_aligned_v282/x36_prefix40_seed1301/EXISTENCE_STAGE_TRACE_V282.mat','baselineSummaryPath','RUN/GA/dynamic_topology/evidence/tracking_aligned_v288/x36_prefix2_integration_seed1301/COMMON_LABEL_MIL_V288.mat','maximumTime',2,'outputRoot','RUN/GA/dynamic_topology/evidence/tracking_aligned_v290/x36_prefix2_integration_seed1301'); [p,r]=runFovGaussianSplittingV290(o); disp(p);" 2>&1 | tee RUN/GA/dynamic_topology/evidence/tracking_aligned_v290/x36_prefix2_integration_seed1301/integration.log
```

Save raw candidate output before scoring, and reuse it after observation
timeouts or analysis failures. Do not restart a live or completed filter.
### Frozen 40-step execution

Source checkpoint `702f84d` was committed and pushed before launch. The
40-step candidate is session 87267, Octave PID 25555. Its first filter step
started at 08:53:49 local time on 2026-09-06. At 09:07:23 the process was
still running at 100% CPU, and the log had reached step 21/40 at 09:05:49.
No completed 40-step metrics were available at that observation. Only paper
and record files were edited after launch; the filtering source is frozen.

```sh
set -o pipefail
/opt/homebrew/bin/octave --no-gui --quiet --eval "addpath(genpath(pwd)); o=struct('baselineTracePath','RUN/GA/dynamic_topology/evidence/tracking_aligned_v282/x36_prefix40_seed1301/EXISTENCE_STAGE_TRACE_V282.mat','baselineSummaryPath','RUN/GA/dynamic_topology/evidence/tracking_aligned_v288/x36_prefix40_seed1301/COMMON_LABEL_MIL_V288.mat','maximumTime',40,'outputRoot','RUN/GA/dynamic_topology/evidence/tracking_aligned_v290/x36_prefix40_seed1301'); [p,r]=runFovGaussianSplittingV290(o); disp(p);" 2>&1 | tee RUN/GA/dynamic_topology/evidence/tracking_aligned_v290/x36_prefix40_seed1301/run.log
```

Read the log without restarting the filter:

```sh
tail -f RUN/GA/dynamic_topology/evidence/tracking_aligned_v290/x36_prefix40_seed1301/run.log
```

## Open Issues

The local update and KLA remain approximate; low-weight boundary leaves and
the original posterior truncation are retained. A fixed-route fusion/local
update effect is not evidence of a dynamic-routing contribution.

## Recommendation

The integration is complete and the frozen 40-step candidate is running.
Do not change parameters in response to the two-step metrics. Use the
same development screen as V288: E-OSPA gain >= 1%, lower count error,
non-worse prefix representative disagreement, common-finite RMSE <= 1%
degradation, worst-formation E-OSPA <= 1% degradation, attempted bytes <=
1.05 times reference, and identical attempted/delivered masks. This is a
screening decision only, not the final goal. Report finite coverage and
formation RMSE as well. Do not extend to M24/full episodes merely because
one aggregate metric improves; assess the completed joint tradeoff first.
