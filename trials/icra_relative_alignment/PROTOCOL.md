# Current-observation relative translation experiment

Registered before computing new alignment estimates or corrected tracking
outcomes. This is a new source-model direction under the user's 2026-09-10
request to leave weak evidence-gating refinements promptly. All 14 segments
are exposed development data: the original nine V2V4Real segments and the
five V2X-Real validation segments. No new generalization claim is possible.

## Hypothesis and relationship to prior work

The prior raw-signal diagnostic found substantial inter-source spatial
misalignment in one V2X sequence. It did not identify pose error, clock error,
or annotation error as the cause. This experiment asks whether a common
translation estimated from current LiDAR observations improves tracking.
No truth, detector confidence, identity, existence probability, future frame,
or selected failure location enters the alignment estimator.

Pose consistency in cooperative perception is an established direction.
CoAlign uses an agent-object pose graph; this small occupancy registration
experiment is not an implementation of CoAlign and makes no novelty claim.
See [Lu et al., Robust Collaborative 3D Object Detection in Presence of Pose
Errors](https://arxiv.org/abs/2211.07214). Preserving multi-object fusion
hypotheses is another substantive option, but is deferred while this directly
observed common spatial discrepancy is tested; see [Yi et al., Computationally
Efficient Distributed Multi-sensor Fusion with Multi-Bernoulli Filter](https://arxiv.org/abs/1906.07991).

## One frozen estimator

Use the release's existing transforms into the current first-agent LiDAR
frame. The first agent defines the reference, as in the existing benchmark.
From each current raw point cloud, retain common-frame heights in [-1.5, 3)
m. Build binary occupancy on x in [-50, 50) m and y in [-40, 40) m, with 0.5 m
cells. These grid, height and search choices are inherited from the previous
raw diagnostic, not optimized on corrected tracking scores. No object mask,
ground-truth mask, ray-based visibility label, or detection mask is used.

For every integer translation (sx, sy) in [-20, 20]^2, count intersections
between the first grid and the translated second grid. Outside-grid cells
are empty. Choose the maximum-overlap translation, breaking ties by smallest
squared norm, then sx, then sy. Empty inputs therefore select zero. If the
chosen maximizer touches the search boundary, apply zero and retain the raw
maximizer in the record: the bounded search has not localized the optimum.
Otherwise apply (0.5*sx, 0.5*sy) m to source two. No score threshold, temporal
smoothing, shrinkage parameter, yaw fit, or covariance rescaling is searched.

The FFT implementation must reproduce every one of the 1,681 integer
overlap counts with a separate padded-integer bitset implementation, on
every frame. Synthetic checks include zero shift, positive and negative
shifts, empty and disjoint grids, ties, and a boundary maximum. Occupancy
construction is checked against separate floor/index binning. A separate
homogeneous-coordinate calculation checks projection to 1e-9 m; binning
uses the explicitly specified projection sums to avoid changing cell-side
decisions through floating-point reassociation.

## Complete native experiment and fair comparison

Use reliable communication only in this first experiment. Both agents receive
the two current occupancy grids before the local filter update and can
compute the same translation. This requires an additional communication
phase. There is no use of future data or undelivered information. An
intermittent-link method requires a separate causal protocol and is not
claimed by this experiment.

Translate the already selected source-two measurements and its model sensor
position together. Keep every measurement, its order, mark, likelihood ratio,
birth rule, pD=0.9, dynamics, fusion rule and output threshold as registered
for the original baseline. Keep source one fixed. The original evaluation
positions, spatial crop, truth coordinates and IDs remain the scoring
domain. This preserves the benchmark's set of scored targets; it does not
redefine the task by moving annotations or dropping inconvenient targets.

Run all 14 complete trajectories for both original GCE and No-age with the
same estimated translations (28 corrected native runs). Reuse the verified
complete uncorrected runs for both methods, recomputing their scores. Before
corrected runs, run the new wrapper with exact zero translations for original
V2V sequence 0000 and V2X sequence 0002, both methods (4 native parity runs).
Require exact output, label, count and ordinary-score parity. GCE's original
fusion and local Gaussian records must also reproduce exactly. The full
new GCE distributions, packet accounting, output extraction and scores are
independently reconstructed. No corrected run begins before this parity
gate and the complete raw-feature verification pass.

Preserve every segment result, and report equal-segment means separately
for V2V and V2X, with OSPA, GOSPA and squared localization/missed/false costs.
The eight fixed continuation comparisons are: for both methods, V2X OSPA
and GOSPA must each decrease by at least 1%; V2V OSPA and GOSPA must each
not increase. All eight must pass to justify engineering this estimator
further. Failure closes this exact estimator; it does not justify choosing
individual frames, changing the grid/band, or fitting a favorable offset.
Passing is only exploratory evidence, not a paper-ready contribution.

Separately compare corrected GCE against corrected No-age. A common input
improvement cannot be attributed to GCE's evidence correction. This is a
causal intervention on a proposed measurement-coordinate model, not proof
that physical pose error caused every mismatch in the source dataset.

## Cost, data acquisition and reproducibility

Each 200 by 160 binary grid is bit-packed into 4,000 bytes with a 32-byte
version/source/frame/grid header. The estimator consumes decoded packet
grids. Each frame sends two additional 4,032-byte messages before density
fusion. Under the existing 16 KiB allocation and 128-byte control cost per
message, charge 33,024 extra wire bytes per frame. Report both these extra
costs and the new trajectory's actual density-packet costs. This first
experiment is expected to be expensive and does not claim communication
efficiency, transport-latency compensation, or real-time operation.

Record grid construction and registration wall times separately from raw
file parsing and native tracking. Both receivers solve the registration;
do not count a single centralized solve as two free local computations.
No full raw cloud is transmitted in the simulated alignment phase.

The old raw files and all earlier studies remain immutable. Missing public
V2V PCD entries may be read in bounded ZIP byte ranges. Check each original
entry's size, CRC and new SHA-256, then preserve the compact occupancy and
source manifest without retaining gigabytes of redundant raw files. Existing
raw files are read in place. Public download tokens stay in process memory.
Every compact shard is bound to the frozen code and raw archive directory;
input preparation can resume from verified shards, never from unchecked
partial output. Native outcomes are never overwritten.
