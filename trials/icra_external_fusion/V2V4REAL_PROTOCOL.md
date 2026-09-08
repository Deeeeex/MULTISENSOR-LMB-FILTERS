# V2V4Real detection replay protocol, 2026-09-08

Initial protocol registered before any tracking outcome; the explicit
preflight amendments below precede the complete replay cohort.
This is a two-dimensional, two-vehicle fusion replay, not the V2V4Real 3-D
leaderboard or a reproduction of DMSTrack's learned Kalman filter.

## Data and scoring domain

Use all nine `val` sequences of the DMSTrack release (the paper's evaluation
split): 0000--0008, lengths 147, 114, 144, 198, 180, 310, 304, 221, 375;
1993 paired frames at 10 Hz. Each vehicle's no-fusion detector outputs and
the tracking labels are text files in the pinned author repository. The
detector already applies a 0.20 confidence cutoff; use all released outputs
without another score threshold. Do not fit parameters on these sequences.

The conversion script swaps the KITTI-formatted y/z fields back to planar
LiDAR x/y. Both vehicles' detections are already transformed into the
**current ego vehicle frame** by the release's postprocessor. Do not transform
the second vehicle's boxes twice. Read the archived per-frame 4x4 source-to-
ego transforms for sensor geometry. The ego transform must be identity.
No world poses are included in this minimal release. Track in the moving
ego frame, with the same absence of ego-motion compensation as the released
DMSTrack baseline configuration (`ego_com: false`). The CV model is therefore
an approximation in this frame; do not describe this as world-frame tracking
or claim accurate global robot localization.

For a known, conservative finite sensing region compatible with the existing
filter, crop real detections to a 40 m planar radius around their own source.
This circle fits inside the released single-vehicle detector rectangle
[-70.4,70.4] x [-40,40] m. Also restrict evaluation and observations to that
rectangle in the current ego frame, where annotations are available. Score
the released ground-truth car centers inside this rectangle and the union
of the two 40 m disks. Apply precisely the same region to reported estimates.
Exclude the 3 m planar neighborhood of either sensing vehicle from truth,
measurements and reported estimates. The original labels include the sensing
vehicles themselves while detector preprocessing masks ego-body returns;
their known positions therefore define a platform-exclusion region. This
task-domain exclusion uses poses only and is identical for every arm.
This is an explicit evaluation crop, not the LiDAR's physical maximum range
or an occlusion map. Ground truth is used only for domain selection and
scoring, never for births, measurement generation, association, or motion.

## Shared local backend and independent births

- State [x,y,vx,vy], dt=0.1 s; relative-frame CV; white acceleration intensity
  25 m^2/s^3; measurement covariance I_2 m^2; nominal p_D=0.9 inside the
  crop, zero outside; p_S=0.99 per scan; assumed Poisson clutter rate 3 per
  source per scan and density 3/(pi*40^2). These are fixed likelihood-model
  assumptions, not empirical claims about detector calibration.
- LBP-LMB local update. Moment-project each updated Bernoulli to one Gaussian
  so the cross-node Gaussian symmetric-KL matching cost is exact for the
  transmitted representation. Include association spread in that projection.
  Prune r<=0.001. A resource guard at 2,000 Bernoullis stops the run, with no
  silent component truncation. MAP cardinality extraction
  is shared by all density arms.
- Each source independently creates a Bernoulli at every **previous-frame**
  detection: r_B=0.01, zero initial velocity, covariance diag(16,16,225,225).
  Labels are (current birth scan, source ID and detection index), with no
  shared truth IDs, target birth times or future measurements. Measurement-
  generated births have genuine observation history from the preceding scan.
  A new birth is tested by the current measurement update only once.
- Pairwise label matching uses the public Gao author manuscript's Section
  V-B augmented assignment, including an unmatched cost of T_D=50 for each
  side. Use its Eq. (54) symmetric KL (called JSD in that manuscript) between
  full-state Gaussians. Same known labels are preserved first; the remaining
  unmatched labels use optimal assignment with private dummy slots. Reuse
  this exact alignment for ER, ER without age, and MIL. A receiver preserves
  its own label when matching a new remote label, and directly observed
  timestamps stay receiver-local after fusion.

## Arms and communication

Local, ER without age, ER, and FoV-MIL with label assignment (MIL-AM).
MIL-AM fuses represented common/exclusive label subspaces according to the
public manuscript's constrained LMB-MIL formulas; arithmetic spatial mixtures
are moment-projected after fusion. It is a documented implementation of
arXiv:1911.01083v1, not a verified copy of closed TAES 2022 source code.
TC-OSPA2 windows 5 and 10 use the same Local outputs and the unchanged author
kinematic functions, with no density feedback and no shared-ID shortcut.

Run each sequence under two predeclared radio conditions: reliable (both
directions every scan), and intermittent (independent directed 10% packet
loss plus a bidirectional outage during the middle 20% of each sequence,
frames floor(0.4*T)+1 through floor(0.6*T), inclusive). Use seed 8301+sequence
index for the complete directed-uniform array; reuse it for every arm.
These are emulated radio conditions, not recorded V2V4Real link outages.

One packet opportunity per direction/scan. Equal 0.5/0.5 fusion weights
when received, self-only when lost. ER age remains 0.25+0.75 exp(-age/5 s),
exactly the case-study rule. Serialize every actual scalar, full-state
covariance, ID and age field and require a lossless round trip. Publish raw
payload bytes and 16 KiB-fragment slot cost plus 128 B/node/frame modeled
control. Geometry transforms are shared common input; disclose that this
fixed pose-information cost is not included in posterior payloads.

## Reporting and stop rules

Recompute planar OSPA (p=2,c=12 m), count MAE, GOSPA localization/missed/false
components, and matched error from saved output states using an independent
assignment implementation. Report every sequence and both radio conditions,
macro-averaged across the nine sequences; frames and nodes are not independent
replicates. Provide paired descriptive differences and 95% bootstrap intervals
over sequences (10,000 resamples, seed 8301), noting only nine sequence units
and potentially related driving routes. Include all adverse results.

Check timestamps, count bounds, finite states, SPD covariances, identity
matrices, assignment behavior, scoring-domain consistency, and absence of
truth input to the tracker. Correct implementation errors for all affected
arms; do not tune model parameters, crop, birth/extraction rules or ER weights
from replay outcomes. Existing synthetic event families become mechanism case
studies, with their no-new-target negative control preserved.

Sources: DMSTrack `docs/DATA.md`, `DMSTrack/configs/v2v4real.yml`,
`V2V4Real/opencood/tools/inference.py`, `infrence_utils.py`,
`data_utils/post_processor/voxel_postprocessor.py`, and the V2V4Real CVPR 2023
paper (arXiv:2303.07601). Exact commits and transform CRC/hash records are
stored with the dependency and data manifests.

## Preflight amendments (before the complete nine-sequence cohort)

The first attempt stopped on sequence 0003 when its 100-Bernoulli resource
assertion fired. No pruning-to-100 was performed. Raise this resource guard
to 2,000 without changing any posterior, birth, likelihood or fusion rule;
record actual maximum counts and fragment costs. Inspection of source labels
also identified the platform self-label issue described above (e.g. labels
within 1 m of the ego origin and the second sensor position in sequence
0002). Apply the uniform 3 m platform exclusion to all three data paths and
rerun every sequence/arm, including already completed sequences. The first
attempt is kept in the run log; it is not included in reported aggregates.
No outcome-based parameter tuning was performed. These are task-domain and
resource corrections after preflight, not a pristine unseen-data validation.
An additional integration check found that the core circular-FoV model did
not implement the replay rectangle and platform exclusions. A replay-only
quality adapter now applies the identical crop in local likelihoods and
fusion absence tests; its captured base function is checked against the
unchanged core. Set absence-existence bounds to the declared pruning/
transmission threshold 0.001 (the reused case-study configuration defaulted
to 0.01). The interrupted domain-integration attempt is logged separately;
all final replay outputs are regenerated under this consistent definition.
