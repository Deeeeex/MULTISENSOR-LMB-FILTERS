# V226 lossless GM canonicalization finding

## Question

Can exact Gaussian-mixture representation redundancy be removed before LMB
mixture reduction so that the installed KLA path is representation-independent,
retains distinct modes under a fixed component budget, and sends fewer full
posterior bytes without changing the represented density?

Decision supported: whether to freeze lossless GM canonicalization as part of
the corrected KLA baseline before resuming X36 routing experiments.

## Scope

- Branch: `codex/v218-stratified-sampling`.
- Code paths:
  `multisensorLmb/canonicalizeLmbGaussianMixtureRepresentation.m`,
  `lmb/computePosteriorLmbSpatialDistributions.m`, and
  `multisensorLmb/fuseLmbPosteriorsByLabel.m`.
- Focused invariant check:
  `test_canonical_lmb_gm_representation.m`.
- Legacy development snapshots from X36 seed 1301 at `t=118,132` and M24
  seeds 1301/1303 at `t=119,131,104,132`.
- Existing two-sensor crossing smoke and powered-GM grid calibration.

Excluded: the legacy snapshots do not provide a corrected full-trajectory
tracking comparison, the byte calculation is a one-message-per-sensor proxy,
and this package does not establish a deployable routing gain.

## Risk Tier

`L2` for the code correction and reproducible mechanism checks. Any use as a
paper contribution or M24/X36 performance claim remains `L3` and requires a
fresh same-version baseline, candidate, and author review.

## Claims

| ID | Claim | Confidence | Evidence IDs | Caveats |
|:--|:--|:--:|:--|:--|
| C1 | Merging bit-identical Gaussian components by summing their weights is invariant to exact component splitting and preserves the represented density when no positive-weight component is pruned or truncated. | High | E1, E2 | This does not merge merely similar components. |
| C2 | Canonicalizing before pruning and the component cap prevents duplicate copies from crowding out a distinct lower-weight mode. | High | E1, E2 | The check is synthetic but directly exercises the production posterior constructor. |
| C3 | Exact-copy redundancy is material in the inspected legacy M24/X36 states: canonical component counts are 79.66% to 84.97% lower. | High | E3 | These are selected development states, not an unbiased trajectory average. |
| C4 | For the legacy X36 seed-1301 `t=132` state, a one-full-message-per-sensor payload proxy falls from 1,350,504 B to 221,040 B, a reduction of 83.63%, without lossy GM approximation. | High | E1, E4 | It is a payload mechanism proxy, not delivered or attempted bytes over a full run. |
| C5 | The integration does not change the controlled powered-GM calibration result and executes the existing two-sensor filter smoke end to end. | Medium | E5, E6 | These checks do not replace M24/X36 same-version reruns. |

## Evidence Ledger

| ID | Type | Source or artifact | What it supports | Strength |
|:--|:--|:--|:--|:--:|
| E1 | code | The three code paths in Scope merge exact copies before reduction and reuse the same public canonicalizer at the KLA boundary. | C1, C2, C4 | strong |
| E2 | command | `octave --no-gui --quiet --eval "addpath(genpath(pwd)); which canonicalizeLmbGaussianMixtureRepresentation; which computePosteriorLmbSpatialDistributions; test_canonical_lmb_gm_representation;"` returned `Canonical LMB-GM representation tests passed.` The two checks compare an exact split against its unsplit density and verify that a 0.25-weight distinct mode survives a two-component cap after two identical modes merge to weight 0.75. | C1, C2 | strong |
| E3 | command / legacy snapshots | A direct load-and-canonicalize pass reported: X36 `t=118` 7,812 to 1,250 (84.00%); X36 `t=132` 7,912 to 1,189 (84.97%); M24 seed-1301 `t=119` 4,292 to 873 (79.66%); `t=131` 3,650 to 605 (83.42%); M24 seed-1303 `t=104` 5,734 to 1,086 (81.06%); `t=132` 3,726 to 693 (81.40%). | C3 | strong |
| E4 | command / payload estimator | Loading the legacy X36 seed-1301 `t=132` snapshot, applying `estimateLmbPayloadSize` with `xDimension=4`, `eventType=2`, and one message per sensor gave `components 7912 -> 1189; bytes 1350504 -> 221040; reduction 83.63%`. | C4 | strong |
| E5 | command | `octave --no-gui --quiet --eval "addpath(genpath(pwd)); runPoweredGmKlaGridCalibration(false);"` returned powered-GM TV `0.0334`, KL `0.0059`, and existence error `0.0048`, matching the prior corrective calibration. | C5 | strong |
| E6 | command | `octave --no-gui --quiet --eval "addpath(genpath(pwd)); runMixtureAwareHeavyFusionCrossingSmoke(7,false);"` completed all three arms; the mixture-aware arm reported mean E-OSPA `3.4510`, crossing E-OSPA `3.1737`, and `507520` bytes. | C5 | medium |

## Verification Record

Independence status: `self-check only`.

- `git diff --check` passed before packaging.
- The focused invariant check uses the production canonicalizer and production
  posterior constructor rather than a duplicate test-only implementation.
- Falsification attempt: the powered-GM grid calibration was rerun to detect a
  change in the downstream mixture-aware KLA approximation; its recorded
  values were unchanged.
- Integration attempt: the existing real-filter crossing smoke completed all
  three communication arms without a runtime error.
- No independent verifier has reviewed the code or rerun the commands, so this
  package remains draft evidence for author review.

## Risk and Escalation

If canonicalization is applied after truncation, or if it merges non-identical
components, it can silently alter tracking estimates. The implementation
therefore uses exact equality only, performs the merge before pruning/capping,
rejects non-finite or negative weights, and reports whether any normalized
positive weight was discarded. Fresh trajectory results must be generated
from this code line; pre-correction outcomes cannot serve as paired baselines.

## Reproducibility

Run from:

```text
/Users/dex/.config/superpowers/worktrees/MULTISENSOR-LMB-FILTERS/v218-stratified-sampling
```

Core checks:

```bash
git diff --check
octave --no-gui --quiet --eval \
  "addpath(genpath(pwd)); test_canonical_lmb_gm_representation;"
octave --no-gui --quiet --eval \
  "addpath(genpath(pwd)); runPoweredGmKlaGridCalibration(false);"
octave --no-gui --quiet --eval \
  "addpath(genpath(pwd)); runMixtureAwareHeavyFusionCrossingSmoke(7,false);"
```

Snapshot roots:

```text
/Users/dex/.config/superpowers/worktrees/MULTISENSOR-LMB-FILTERS/v152-safe-graph-oracle/RUN/GA/dynamic_topology/evidence/tracking_aligned_v214/trajectory_collection/training
```

The snapshot count and byte-proxy commands load each `*_sig*.mat` file, select
`behaviorBundle.posteriorSnapshots{t}`, apply
`canonicalizeLmbGaussianMixtureRepresentation` to every sensor posterior, and
sum the diagnostics or `estimateLmbPayloadSize` outputs.

## Open Issues

- Measure attempted and delivered bytes in a fresh corrected X36 trajectory;
  the current 83.63% value is only a full-message representation proxy.
- Measure how canonicalization changes mode retention, fallback frequency,
  E-OSPA, RMSE, and consensus over complete M24/X36 trajectories.
- Profile the exact-equality merge cost if richer scenes create substantially
  larger per-label mixtures.
- Obtain independent code and evidence review before using this as a
  publication claim.

## Recommendation

Accept C1-C4 as a draft mechanism finding and freeze lossless canonicalization
as part of the corrected KLA baseline. Start a fresh X36 same-version baseline
before implementing receiver-specific partial transfer. Treat C5 only as an
integration check, and do not promote tracking or communication performance
until the full corrected trajectory is available.
