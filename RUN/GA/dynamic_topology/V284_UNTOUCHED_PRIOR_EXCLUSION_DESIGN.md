# V284: causal startup-prior intervention

Status: the 40-step paired screen completed and failed its joint criterion
on RMSE. E-OSPA and disagreement improved strongly at nearly unchanged
attempted bytes. See the completed result interpretation; no startup-only
parameter sweep or full-episode promotion is authorized by this result.
L2 exploratory, self-check only; no new full-episode result.

## Question

Does mixing never-updated birth priors into the first useful observation
states cause a lasting target-set deficit? V283 cannot exclude that effect:
its late opportunity flags describe the reference trajectory, not a trajectory
in which early prior handling was changed. Test one intervention, not an
existence-threshold, eta-floor, FoV-angle or weight-parameter sweep.

## Intervention and accounting

Each retained Bernoulli carries a Boolean `hasObservationLineage`, false at
birth. A scheduled local update marks it true whenever the weighted detection
probability at the predicted mixture-component means is positive. A missed
detection also counts; no measurement-to-truth matching is used. Prediction
keeps the flag, ordinary pruning removes it with the state, and a receiver
propagates it only through an actually participating positive-weight input.
An active observable-absence censor is informed negative evidence.

If a label has at least one informed existence input, exclude its never-informed
inputs from both spatial and existence pooling, then normalize the remaining
ordinary weights. Otherwise retain the ordinary fusion rule. If only informed
absences remain, no spatial hypothesis survives; drop the label, consistent
with the shared 0.01 absence bound and subsequent pruning. Do not remove a
historically informed input merely because it is currently outside the FoV.

All other routing, packet handling, mixture approximation and output rules
stay fixed. In particular the existing single-input moment matching stays
unchanged: this experiment does not simultaneously replace the fusion backend.
The flag is removed from the observable routing-policy context and remains
available only to the actual fusion/payload path. The geometry-only V242
policy therefore does not acquire a new posterior-information input.
The extra Boolean is charged as one eight-byte scalar per transmitted label
under the repository's existing scalar payload model, on failed attempts too.
There are no extra messages. Recursive GM/label counts can still change the
total bytes. No age/source identity or measurement-level duplicate detector
is introduced; this is a startup-semantic contrast, not the final evidence-age
algorithm or a new theoretical contribution.

## Why the intervention is not guaranteed to help

For a label whose informed and untouched inputs both carry positive weight,
let `u` be the total untouched weight and let `L_I` and `L_U` be the separately
renormalized mean log odds of those two groups. In one frozen input pool,

```text
ordinary log odds = (1-u) L_I + u L_U + log eta_all
restricted log odds = L_I + log eta_informed
difference = u (L_I - L_U) + log eta_informed - log eta_all.
```

Removing weak priors helps the first term when `L_I > L_U`, but the spatial
overlap term also changes. Missed-detection evidence may make `L_I` small, and
the recursive trajectories change after this one fusion. Higher existence
also does not by itself prove correct target identities, lower RMSE, or lower
communication. This elementary KLA identity motivates the paired test; it is
neither a new theorem nor a guarantee of full-episode tracking improvement.

## Paired protocol and stopping condition

- X36 formation-braid, seed 1301, original 160-step inputs truncated to 1--40.
  Reuse the V282 reference trace; no reference filter rerun.
- Use the same filter seed, measurements, physical route and directed packet
  uniforms. Record planned and delivered route correspondence separately from
  output metrics, which are expected to change.
- First check a few direct semantic cases and a two-step integration. The
  integration is not a performance result. Freeze source before the 40-step run.
- Report E-OSPA, absolute count error, conditional and common-finite-cell RMSE,
  mean disagreement among the same six representatives over this entire
  prefix, attempted bytes including metadata, and per-formation/sensor tails.
  Prefix disagreement is not the paper's t=40--140 focus metric.
- A promising prefix requires at least 1% E-OSPA improvement, lower count error,
  no worse representative disagreement, at most 1% common-cell RMSE degradation,
  at most 5% attempted-byte growth versus the already sparse reference, and
  no formation E-OSPA degradation over 1%. This is only a screening rule for
  a full-episode follow-up, not the joint M24/X36 success criterion.
- If the screen fails, retain the result in the repository and do not launch
  a parameter grid or another startup-only variant. If it passes, full-episode
  comparisons must include the same semantic rule on fixed routing before any
  improvement is attributed to dynamic routing. Independent seeds and M24
  remain necessary. A single new arm cannot establish routing novelty.

## Literature boundary

Wang et al. (2018, DOI 10.1016/j.sigpro.2018.04.010) already use per-label
information-based weighting; the public RMIT record currently has no attached
full text (`https://api.figshare.com/v2/articles/27510990`, `files: []`).
OpenAlex's green-OA flag did not establish full-text availability. V284 is not
a claimed replication of Wang or Li's multi-view LMB algorithms.

Gao et al., *Fusion of labeled RFS densities with minimum information loss*,
`https://arxiv.org/abs/1911.01083`, Section V-C, explicitly discusses redefining
local label spaces as information from other views is acquired. That supports
distinguishing current visibility from historical access, not the effectiveness
of this particular intervention. The fusion family in that work is MIL, not
the KLA backend used here. Neither source proves V284 new or beneficial.

## Reproduction

From the ICASSP worktree, check the few semantic cases:

```sh
octave --no-gui --quiet --eval "addpath(genpath(pwd)); checkUntouchedPriorExclusionV284();"
```

Observed output: `V284 semantic self-check PASS: opportunity history,
participation, fallback and byte accounting.` This is not independent validation.
The first integration stopped at step 1 because the routing-context schema did
not allow the new fusion-only field. The correction admits a logical scalar
in the live object, then strips it at that boundary; it does not expose the flag
to the policy. The original failure log is retained beside the integration run.
The corrected integration exited 0 (22.2 s filtering): 92/91 attempted/delivered
messages match the reference exactly, with zero differing edge-time entries.
It exercised 182 excluded source-label pools and charged 17,664 metadata bytes.
This development integration used the uncommitted implementation based on
`fd5722f`; the subsequent source-freeze commit contains that implementation.
Its two-step output metrics are retained only as integration artifacts and
were not used to change the already specified 40-step screen.

Create the output directory first, then use this paired 40-step command after
the corrected integration passes and source is committed:

```sh
mkdir -p RUN/GA/dynamic_topology/evidence/tracking_aligned_v284/x36_prefix40_seed1301
set -o pipefail
octave --no-gui --quiet --eval "addpath(genpath(pwd)); options=struct('baselineTracePath','RUN/GA/dynamic_topology/evidence/tracking_aligned_v282/x36_prefix40_seed1301/EXISTENCE_STAGE_TRACE_V282.mat','maximumTime',40,'outputRoot','RUN/GA/dynamic_topology/evidence/tracking_aligned_v284/x36_prefix40_seed1301'); runUntouchedPriorExclusionV284(options);" 2>&1 | tee RUN/GA/dynamic_topology/evidence/tracking_aligned_v284/x36_prefix40_seed1301/run.log
```

For integration only, use `maximumTime=2` and
`x36_prefix2_integration_seed1301`. A saved raw output makes subsequent
invocation analysis-only. The progress message reports the start of a step,
not its completion. The generator constructs all 160 original steps before
cropping; that setup is additional to the reported filter elapsed time.

```sh
tail -f RUN/GA/dynamic_topology/evidence/tracking_aligned_v284/x36_prefix40_seed1301/run.log
```
