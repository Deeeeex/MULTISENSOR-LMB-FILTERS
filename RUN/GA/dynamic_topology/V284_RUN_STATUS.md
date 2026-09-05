# V284 implementation checkpoint

## Question

Does exclusion of never-informed priors warrant a full-episode method comparison?

## Scope

One X36 seed, a two-step integration and a planned 40-step paired screen.
No new M24 run, full-episode result, paper-table update or novelty claim.

## Risk Tier

L2 exploratory implementation and experiment. No publication/submission action.

## Claims

| ID | Claim | Evidence | Limit |
| --- | --- | --- | --- |
| C1 | The implemented flag survives the filter/fusion path without changing the integration's scheduled or delivered edges. | E1, E2 | Two steps only; self-check. |
| C2 | The actual candidate source is committed and pushed; its 40-step screen has started. | E3, E4 | Launch is not completion or an accuracy finding. |

## Evidence Ledger

- E1: `octave --no-gui --quiet --eval "addpath(genpath(pwd)); checkUntouchedPriorExclusionV284();"` exited 0 with `V284 semantic self-check PASS: opportunity history, participation, fallback and byte accounting.`
- E2: the integration command is the documented V284 command with `maximumTime=2` and `x36_prefix2_integration_seed1301`; session 3128 exited 0. The saved report gives 92/91 attempted/delivered messages and zero differing edge-time entries; filtering took 22.2 seconds. It used the uncommitted implementation based on `fd5722f`, captured by E3.
- E3: source-freeze commit `d9c42fd`, `Test untouched-prior exclusion in label-wise fusion`. `git push` exited 0 with `fd5722f..d9c42fd codex/icassp2027-sparse-causal-routing -> codex/icassp2027-sparse-causal-routing`.
- E4: session 45950 executes the exact 40-step command in `V284_UNTOUCHED_PRIOR_EXCLUSION_DESIGN.md`. At this checkpoint the log includes `Filter starting step 11/40 at 2026-09-06 04:35:03`; this announces the start, not completion, of step 11. Runtime sources have not changed since E3.

## Verification Record

Self-check only. The initial integration schema rejection was corrected by
stripping the fusion-only flag from routing inputs; the corrected integration
completed. No independent verification or full-episode performance check.
The plotting script passed Python syntax compilation, but its actual data
render remains pending; no generated figure is claimed here.

## Risk and Escalation

Do not infer routing novelty from a fusion-rule change. A promising prefix
still requires the same semantic rule on fixed routing, full episodes and M24.

## Reproducibility

The V284 design retains the complete command, baseline trace path, crop rule,
filter seed and fixed screening conditions. The unchanged V282 reference is
reused. Raw MAT files and run logs remain local, with numerical Markdown/CSV
reports tracked. Plot export reads completed results without rerunning filters.

## Open Issues

The 40-step outcome, per-formation tails and complete joint tradeoff are pending.
No new result enters the canonical best-method table until it exists.

## Recommendation

Await the bounded screen, then either close startup-only variants or launch
the specified fixed-routing control. Do not tune the screen after seeing it.
