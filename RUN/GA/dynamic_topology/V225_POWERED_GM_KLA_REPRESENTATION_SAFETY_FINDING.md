# V225 powered-GM KLA representation-safety finding

## Question

Why did the X36 `t=132`, source-12, label `[7,5]` dominant-edge
replacement fail with an invalid eta request, and what correction preserves
the intended label-wise KLA semantics?

Decision supported: whether V225/V226 topology experiments may continue on
the installed mixture-aware LMB-KLA implementation.

## Scope

- Repository branch: `codex/v218-stratified-sampling`.
- Corrective commit: `9dffa3c`.
- Actual X36 posterior slice: `x36-formation-fov`, seed `1301`, `t=132`,
  beneficiary formation 1, label `[7,5]`.
- Code paths:
  `multisensorLmb/fuseLmbPosteriorsByLabel.m`,
  `common/evaluateLabelKlaEtaRetentionV223.m`, and
  `common/projectLabelKlaExistenceRetentionByEtaV223.m`.
- Controlled two-dimensional grid oracle:
  `RUN/GA/runPoweredGmKlaGridCalibration.m`.
- Primary methodological source: Ajgl, Simandl, and Dunik,
  *Approximation of Powers of Gaussian Mixtures*, FUSION 2015,
  <https://c4i.gmu.edu/~pcosta/F15/data/fileserver/file/472209/filename/Paper_1570104849.pdf>.

Excluded: this finding does not establish a tracking gain for V225, does not
validate a deployable routing policy, and does not refresh any full-trajectory
M24/X36 baseline.

## Risk Tier

`L2` for the code correction and reproducible diagnostic; any paper-facing
claim remains `L3` and requires author review plus fresh paired experiments.

## Claims

| ID | Claim | Confidence | Evidence IDs | Caveats |
|:--|:--|:--:|:--|:--|
| C1 | The pre-fix powered-GM path produced impossible positive `log eta` values for the actual S12 route. | High | E1, E2 | This diagnoses the installed approximation, not exact KLA. |
| C2 | Exact duplicate GM components made the approximation depend on an arbitrary component split. | High | E2, E3, E4 | Exact-copy collapse does not solve all errors from overlapping but distinct components. |
| C3 | The corrective path is invariant to exact component splitting and rejects any remaining `eta > 1` approximation by falling back to moment-matched Gaussian KLA. | High | E3, E4, E5 | The fallback is an approximation and is recorded explicitly. |
| C4 | Existing pre-correction tracking outcomes are not paired baselines for post-correction candidates. | High | E3, E6 | Fresh full-history baselines are required before promotion. |

## Evidence Ledger

| ID | Type | Source or artifact | What it supports | Strength |
|:--|:--|:--|:--|:--:|
| E1 | command / actual posterior | Direct reconstruction before `9dffa3c`: R1 `log eta=+0.025536326942`, R2 `+0.068340107392` for the S12 full transfer. | C1 | strong |
| E2 | actual posterior | S1 label `[7,5]` contained eight bit-identical components of weight `0.125`; S12 contained repeated groups of three identical components. | C1, C2 | strong |
| E3 | code / commit | `9dffa3c Make powered-GM KLA representation safe` collapses exact copies, applies the Holder-invariant fallback, and records raw eta plus the fallback reason. | C2, C3, C4 | strong |
| E4 | scripted check | A three-component source and an exactly density-equivalent four-component split source produced `dr=0`, `dlogeta=0`, `dweights=0`, `dmean=0`, `dcov=0`; both retained eight fused components. | C2, C3 | strong |
| E5 | command / actual posterior | Post-fix S12 reconstruction: R1/R3-R6 had effective `log eta=-0.5005/-0.6687.../-0.5756`; R2 recorded raw `+0.418960` and fell back to effective `-0.533029`. | C3 | strong |
| E6 | provenance | The V220 donor capture and frozen reference outcomes were generated before corrective commit `9dffa3c`. | C4 | strong |
| E7 | paper | Ajgl et al. show that fractional powers of arbitrary GMs are not closed and that separated-component approximations have limited applicability. | C1, C2 | medium |

## Verification Record

Independence status: `self-check only`.

- Reconstructed the failing label directly from saved local posterior
  snapshots rather than inferring the failure from the wrapper error.
- Checked the mathematical invariant independently of the implementation:
  for normalized densities and normalized nonnegative fusion weights,
  Holder's inequality gives
  `eta = integral product_i p_i(x)^w_i dx <= 1`.
- Falsification attempt: reran the existing 81-to-161 point grid calibration.
  Its convergence and candidate errors were unchanged after the correction:
  `log eta` grid change `2.263e-11`; powered-GM TV `0.0334`, KL `0.0059`,
  existence error `0.0048`.
- Remaining limitation: there has been no independent verifier, and the
  exact four-dimensional eta for the actual X36 posterior was not numerically
  integrated. The positive-eta contradiction itself does not require that
  integration.

## Risk and Escalation

If this correction is wrong, all subsequent Bernoulli existence probabilities
and paper-facing topology comparisons may be biased. Therefore:

- retain raw eta and fallback-reason diagnostics;
- regenerate corrected full-history references instead of reusing frozen
  pre-correction outcomes;
- treat all post-correction screens against old references as development
  diagnostics only;
- require author review before changing the manuscript's KLA description.

## Reproducibility

Core calibration command:

```bash
octave --no-gui --quiet --eval \
  "addpath(genpath(pwd)); runPoweredGmKlaGridCalibration(false);"
```

Post-fix checks were run from:

```text
/Users/dex/.config/superpowers/worktrees/MULTISENSOR-LMB-FILTERS/v218-stratified-sampling
```

Actual-posterior reconstruction source:

```text
/Users/dex/.config/superpowers/worktrees/MULTISENSOR-LMB-FILTERS/v152-safe-graph-oracle/RUN/GA/dynamic_topology/evidence/tracking_aligned_v220/donor_capture/training/x36_formation_fov_seed1301_t132_f5/screen/FORMATION_MODE_H3_X36_FORMATION_FOV_SEED1301_T132.mat
```

The split-invariance and actual-posterior commands are preserved in the Codex
run that produced corrective commit `9dffa3c`; their representative outputs
are E4 and E5 above.

## Open Issues

- Quantify how often the invariant fallback fires in freshly corrected M24
  and X36 full trajectories.
- Refresh the no-routing/static and full-payload reference outcomes under the
  corrected fusion implementation.
- Add an upper log-odds trust region: the current V223 lower-retention gate
  allowed S9 to increase receiver existence from roughly `0.52-0.63` to
  `0.97-0.98`, which is consistent with the observed RMSE regression.
- Evaluate receiver-wise partial edge transfer only after the corrected
  reference is available.

## Recommendation

Accept C1-C3 as a draft implementation finding and retain commit `9dffa3c`.
Do not promote V225 performance numbers. Following C4, regenerate the X36
reference first, then evaluate a V226 receiver-wise maximum partial transfer
inside a two-sided existence-log-odds trust region.
