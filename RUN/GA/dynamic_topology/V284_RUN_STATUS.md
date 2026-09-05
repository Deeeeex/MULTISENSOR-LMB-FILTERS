# V284 completed prefix screen

## Question

Does exclusion of never-informed priors warrant a full-episode method comparison?

## Scope

One X36 seed, a two-step integration and a completed 40-step paired screen.
No new M24 run, full-episode result, paper-table update or novelty claim.

## Risk Tier

L2 exploratory implementation and experiment. No publication/submission action.

## Claims

| ID | Claim | Evidence | Limit |
| --- | --- | --- | --- |
| C1 | The implemented flag survives the filter/fusion path without changing the integration's scheduled or delivered edges. | E1, E2 | Two steps only; self-check. |
| C2 | The frozen candidate completed its 40-step screen. | E3, E4 | One opened seed, not a full episode. |
| C3 | E-OSPA improves 13.362% and disagreement 20.428%, but official RMSE rises 164.715%; the joint screen fails. | E4, E5 | Attempted bytes rise 0.037%; no uniform or statistically significant gain claim. |
| C4 | For common geometrically assigned targets pooled RMSE changes 8.730 -> 8.898 m; newly assigned targets have 34.731 m pooled RMSE. | E6 | Post-hoc matching-set composition, not confirmed label identity or a replacement for the official metric. |

## Evidence Ledger

- E1: `octave --no-gui --quiet --eval "addpath(genpath(pwd)); checkUntouchedPriorExclusionV284();"` exited 0 with `V284 semantic self-check PASS: opportunity history, participation, fallback and byte accounting.`
- E2: the integration command is the documented V284 command with `maximumTime=2` and `x36_prefix2_integration_seed1301`; session 3128 exited 0. The saved report gives 92/91 attempted/delivered messages and zero differing edge-time entries; filtering took 22.2 seconds. It used the uncommitted implementation based on `fd5722f`, captured by E3.
- E3: source-freeze commit `d9c42fd`, `Test untouched-prior exclusion in label-wise fusion`. `git push` exited 0 with `fd5722f..d9c42fd codex/icassp2027-sparse-causal-routing -> codex/icassp2027-sparse-causal-routing`.
- E4: session 45950 executed the exact 40-step command in `V284_UNTOUCHED_PRIOR_EXCLUSION_DESIGN.md` and exited 0: `V284 prefix 40: E-OSPA 135.180030 -> 117.117031 (+13.362% gain), bytes ratio 1.0004, screen evaluated 1 / passed 0.` Raw filtering took 703.8 s; the subsequent assignment-metric calculation took additional time. Runtime sources were not changed during the run. The report, per-sensor CSV, raw MAT and run log are in `evidence/tracking_aligned_v284/x36_prefix40_seed1301/` below this directory.
- E5: `octave --no-gui --quiet --eval "addpath(genpath(pwd)); exportUntouchedPriorFigureV284('RUN/GA/dynamic_topology/evidence/tracking_aligned_v284/x36_prefix40_seed1301/UNTOUCHED_PRIOR_EXCLUSION_V284.mat');"` returned `V284 figure data exported: 40 steps, 5 joint metrics; no filter rerun.` Then `/Users/dex/miniconda3/bin/python3 RUN/GA/plot_untouched_prior_v284.py RUN/GA/dynamic_topology/evidence/tracking_aligned_v284/x36_prefix40_seed1301` exited 0 and exported SVG/PDF/PNG plus the numeric manifest. The PNG was visually inspected; all plotted values come from the unrounded CSVs.
- E6: `octave --no-gui --quiet --eval "addpath(genpath(pwd)); analyzeUntouchedPriorMatchedTargetsV284('RUN/GA/dynamic_topology/evidence/tracking_aligned_v284/x36_prefix40_seed1301/UNTOUCHED_PRIOR_EXCLUSION_V284.mat');"` exited 0 (session 82371): `V284 common-target pooled RMSE 8.730201 -> 8.897792; added 34.730995; original screen remains failed.` All 2,880 per-cell RMSE values agree with the original solver's saved values within 1.42e-14 m. The same analysis reuses the cached static prefix, whose official RMSE is 8.753521 m; V284 also loses on RMSE against that original-fusion static reference.

## Verification Record

Self-check only. The initial integration schema rejection was corrected by
stripping the fusion-only flag from routing inputs; the corrected integration
completed. The paired filter, aggregate metrics, per-formation results and
post-hoc matched-target analysis are now complete. No independent verification
or full-episode performance check. The rendered figure is an internal
experiment record; it has not been inserted into the paper or Lark best table.
Command `python3 /Users/dex/.codex/skills/auto-research/scripts/evidence_lint.py RUN/GA/dynamic_topology/V284_RUN_STATUS.md`
returned `PASS: RUN/GA/dynamic_topology/V284_RUN_STATUS.md` at this handoff.

## Risk and Escalation

Do not infer routing novelty from a fusion-rule change. A promising prefix
still requires the same semantic rule on fixed routing, full episodes and M24.

## Reproducibility

The V284 design retains the complete command, baseline trace path, crop rule,
filter seed and fixed screening conditions. The unchanged V282 reference is
reused. Raw MAT files and run logs remain local, with numerical Markdown/CSV
reports tracked. Plot export reads completed results without rerunning filters.

## Open Issues

The position-error cost of newly represented targets remains unresolved,
especially in formations 5 and 6. Same-label accurate-source availability has
not yet been established. Fixed routing with the same V284 semantics, M24,
full episodes and independent seeds remain untested; static prefix bytes are
not available in the cached summary. No joint-goal success is claimed.

## Recommendation

Close startup-only tuning under the frozen rule. Preserve the substantial
set-recovery signal as a method-design finding, not a successful final method.
Next check whether accurate same-label spatial information exists elsewhere
in the saved network state before designing a costed delivery action. Do not
replace the official RMSE with the favorable common-target subset.
