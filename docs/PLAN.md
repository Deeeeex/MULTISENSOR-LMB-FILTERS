# Plan: FID-FIA-Informed Existence Refinement

## Summary
Create `docs/FID_FIA_EXISTENCE_REFINEMENT_PLAN_CN.md` and implement a new hybrid adaptive-weight variant whose goal is: **all three main consensus metrics beat the current Cao-Zhao FID-FIA baseline**, while preserving the current method’s spatial/RMSE strengths as much as possible.

The design should not replace the current method. It should add a new experiment arm that keeps the existing factorized/structure-aware spatial branch, and injects FID-FIA only into the existence/cardinality branch.

## Key Changes
- Add a new config path in `computeAdaptiveFusionWeights.m`:
  - `useFidFiaExistence = true`
  - `fidFiaExistenceStrength = 0.5` as default
  - `fidFiaExistencePower = 1.0`
  - `fidFiaExistenceMinScore = 0.4`
  - reuse existing FID-FIA helpers: posterior m-projection, pairwise FID accumulation, existence-weighted target pairs, detection/FoV-aware Fisher metric.
- Apply FID-FIA only to `existenceScore`, not `spatialScore`:
  - keep current `spatialScore` formula unchanged
  - compute normalized `fidFiaScore`
  - map it to bounded score: `fidFiaExistenceScore = minScore + (1-minScore) * normalizedFia^power`
  - update existence branch by geometric modulation: `existenceScore = existenceScore .* fidFiaExistenceScore^strength`
- Keep current method identity:
  - covariance, link quality, existence confidence, decoupled KLA, and structure-aware priors remain active
  - FID-FIA is presented as an existence/cardinality refinement, not as a replacement method.
- Add a new main-experiment arm in `runMultisensorFilters_formation_4plus4_TieredLinkAblation.m`:
  - `+FID-FIA existence refinement`
  - comparison order: `fixed weights -> Cao-Zhao FID-FIA baseline -> +structure-aware decoupled KLA -> +FID-FIA existence refinement`
  - expose with `finalArmMode='fidFiaExistenceRefinement'`.
- Add debug fields:
  - `debug.fidFiaExistenceScore`
  - `debug.fidFiaScore`
  - `debug.fidFiaPairCounts`
  - `debug.useFidFiaExistence`

## Experiment Plan
- First run a 1-trial smoke test:
  - verify no runtime/numerical errors
  - inspect consensus OSPA/RMSE/Card and local CardErr/RMSE.
- Then run the full matched 20-trial experiment:
  ```matlab
  [reportPath, summary] = runMultisensorFilters_formation_4plus4_TieredLinkAblation( ...
      20, 1, true, struct(), true, 'fidFiaExistenceRefinement');
  ```
- Acceptance target:
  - consensus OSPA < `1.820229`
  - consensus RMSE < `1.647412`
  - consensus Card < `0.126188`
  - local CardErr <= `0.392313`
  - local RMSE should stay close to current method; hard failure if worse than FID-FIA local RMSE `1.715746`.

## Documentation Updates
- Create the requested plan file:
  - `docs/FID_FIA_EXISTENCE_REFINEMENT_PLAN_CN.md`
- After experiments complete, update:
  - `docs/PAPER_MAIN_RESULTS_CN.md`
  - `docs/paper/07_results_and_ablation.md`
  - `docs/paper/els-cas-templates/sections/06_results.tex`
- If the new hybrid beats FID-FIA on all three consensus metrics, promote it as the new headline method.
- If it improves Card but does not beat FID-FIA, keep it as a targeted cardinality-refinement ablation and preserve the current headline method.

## Assumptions
- FID-FIA should enter only the existence/cardinality branch, per user preference.
- The target is ambitious: all three consensus metrics should beat Cao-Zhao FID-FIA. The implementation should make this measurable, but the final paper positioning depends on the 20-trial result.
- No existing FID-FIA baseline behavior should be removed; it remains the external comparison arm.

## Implementation Outcome

Implemented as `finalArmMode='fidFiaExistenceRefinement'`. The tuned main arm keeps the structure-aware spatial branch unchanged, enables `useFidFiaExistence`, and uses `fidFiaExistenceStrength=4.0`, `fidFiaExistenceMinScore=0.0`, `existenceEmaAlpha=0.0`, and `existenceMinWeight=0.0` for the existence branch.

20-trial result from `RUN/GA/GA_TIERED_LINK_ABLATION_N20_SEED1_20260512_155714.md`:

| Arm | Consensus OSPA | Consensus RMSE | Consensus Card | Local RMSE | Local CardErr |
| --- | ---: | ---: | ---: | ---: | ---: |
| Cao-Zhao FID-FIA baseline | 1.820229 | 1.647412 | 0.126188 | 1.715746 | 0.392313 |
| +structure-aware decoupled KLA | 1.785873 | 1.562521 | 0.192938 | 1.598561 | 0.578688 |
| +FID-FIA existence refinement | 1.668961 | 1.528182 | 0.061062 | 1.704538 | 0.221563 |

The hybrid arm beats the Cao-Zhao FID-FIA baseline on all three consensus metrics and on local CardErr. Its local RMSE is worse than the old structure-aware arm but remains slightly better than the FID-FIA baseline.
