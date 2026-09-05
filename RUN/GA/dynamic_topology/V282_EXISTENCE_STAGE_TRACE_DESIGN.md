# V282: early X36 existence-stage trace

Status: COMPLETE. The unchanged 40-step reference capture and offline analysis
exited 0 (session 4122; frozen runtime/driver commit `fbf17cd`). It captured
1,440 receiver-time cells and 34,424 label stages. Every E-OSPA cell, finite
RMSE value and finite-RMSE mask matches the stored reference prefix; maximum
metric differences are zero. This is metric correspondence, not full-state
equivalence or a new method gain. Filter runtime was 609.8 seconds, excluding
scene generation and offline analysis.

At 27,939 zero-component-mean-pD label stages, the maximum absolute local
existence change is 2.22e-16. At step 40, mean predicted/local/pre-spatial/fused
existence masses are 5.946/6.020/6.153/6.062, with local/output MAP counts
5.806/5.861. These are averages over receivers, not true-target recall.
The observation-lineage follow-up and bounded method decision are in
`V283_OBSERVATION_EVIDENCE_FINDING.md`. No new tracking arm is running.

The preceding short run completed in 21.6 seconds of filter time, after full-scene
generation. Across 72 receiver-time cells, E-OSPA and finite RMSE match the
saved reference prefix exactly, including the finite-RMSE mask. It captured
1,728 label stages. At 1,488 zero-expected-pD stages, the maximum absolute
local existence change is 6.94e-18. This supports the narrow local-update
check, not a full-episode explanation of X36's count error.

An initial short run reached the save step but failed because Octave cannot
serialize live model function handles as MAT7. The driver now saves only
the sensor-quality numerical fields needed offline. The corrected short
run exited 0; the 40-step trace uses that same corrected source. The stored
trace is diagnostic data, not a complete restartable filter checkpoint.

## Question and decision

V281 localized weak existence evidence to before the final fusion at three
M24 anchors. It did not separate earlier local updates from repeated fusion
and did not inspect X36. V282 captures X36 steps 1--40 to locate the onset of
that weakness. It is an unchanged-reference diagnostic, not a candidate arm.

The local update already sets detection probability to zero outside its FoV.
No evidence currently justifies a sensing-model or missed-detection code fix.
The existing component-mean FoV approximation remains shared and unchanged.

## Fixed inputs

- Preset: `x36-formation-fov-temporal-coupled-formation-braid`, seed 1301.
- Generate the original 160-step scene, then truncate the existing arrays.
  Do not regenerate a 40-step scene with changed motion/phase timing.
- V242 route, weights, packet handling, powered-GM approximation, local
  association and MAP readout remain unchanged. Use the existing filter seed
  offset and directed delivery uniforms.
- Capture predicted, post-local-update and pre-pruning fused posteriors, plus
  the actual label-wise weighted log odds and spatial overlap term.
- The filter sees no realized target truth. Truth is used only for offline
  metric correspondence with the already saved V274 reference prefix.

## Outputs and stop rule

Save the expensive raw trace before analysis. Produce receiver/time and
label/time CSVs, expected component-mean detection probability, existence
mass by stage, and MAP cardinality. Keep unselected retained labels distinct
from pruned labels. Compare every E-OSPA/RMSE prefix cell with the old result;
do not imply full-state identity merely from matching output metrics.

First run two steps in a separate integration directory. If capture or metric
correspondence fails, stop and resolve that difference before the 40-step run.
After the prefix completes, identify whether local updates, existence pooling
or spatial overlap is the earliest major change. Do not launch a threshold
sweep or a new full episode without that analysis. If 40 steps do not localize
the issue, state the diagnostic limit rather than asserting a cause.

The existing X36 reference took about 10.2 hours for 160 steps. A prefix avoids
spending another full episode before choosing a method change. No change to
the paper's best-method table is warranted by this diagnostic alone.

## Run

From this worktree (set `maximumTime` to 2 and change `outputRoot` to
`x36_prefix2_integration_seed1301` for the one integration check):

```sh
octave --no-gui --quiet --eval "addpath(genpath(pwd)); options=struct('referenceResultPath','/Users/dex/.config/superpowers/worktrees/MULTISENSOR-LMB-FILTERS/v274-x36-minimum-backbone/RUN/GA/dynamic_topology/evidence/tracking_aligned_v274/x36_minimum_backbone_seed1301/minimum_causal_backbone/CAUSAL_MINIMUM_FORMATION_BACKBONE_V242_FULL_EPISODE.mat','maximumTime',40,'outputRoot','RUN/GA/dynamic_topology/evidence/tracking_aligned_v282/x36_prefix40_seed1301'); runExistenceStageTraceV282(options);" 2>&1 | tee RUN/GA/dynamic_topology/evidence/tracking_aligned_v282/x36_prefix40_seed1301/run.log
```

Create the output directory before opening the tee log. The driver reports
the start of every fifth step; a progress line does not mean that step has
completed. Reinvocation after a saved trace only refreshes offline analysis.

Follow the launched run without starting another process:

```sh
tail -f RUN/GA/dynamic_topology/evidence/tracking_aligned_v282/x36_prefix40_seed1301/run.log
```
