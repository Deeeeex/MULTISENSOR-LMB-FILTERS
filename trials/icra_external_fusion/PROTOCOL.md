# External fusion extension, 2026-09-08

Registered before running either new TC arm. Preserve the v4 case-study
inputs, results, and ER parameters. No method tuning using these outcomes.

## TC-OSPA2 on all existing paired cases

- Author repository: https://github.com/AdelaideAuto-IDLab/Distributed-limitedFoV-MOT
  at `b6b20ec30b7854dcee6f4a718237d82d96ac7c2a`. Research-use notice in
  `Readme.md`; source stays in the ignored dependency cache.
- Reuse the **local-only** LMB estimates from all 60 v4 cases, seeds
  2901--2920, three families. The author's method is filter-agnostic. Its
  fusion output is never fed back into the local LMB. The common local
  backend, birth information, MAP extraction, and measurement realization
  are therefore exactly those of the existing Local arm. This is an
  adaptation to our backend, not reproduction of the author's Gibbs-LMB
  experiment or its measurement-adaptive births.
- Use the unchanged author track matching and two-stage multi-neighbor
  kinematic fusion. Independently namespace each node's track labels, so
  TC must match trajectories rather than using a shared-ID shortcut.
- Primary window 5 scans (author scenarios 1--2); fixed sensitivity window
  10 (author scenario 3). As in the author code, unmatched minimum length is
  floor((window-1)/2), order is 1, and consecutive-length filtering is off.
  Matching cutoff is 12 m, the existing problem's evaluation cutoff. Report
  both windows irrespective of their results. No window/cutoff search.
- Preserve the exact directed delivered graph from the saved v4 uniforms.
  A receiver with no delivered neighbor keeps its local output. Each
  delivered packet contains the source's current and trailing-window LOCAL
  estimates. No inaccessible source history or current association state
  is used. Association-history label reconciliation is local; report set
  metrics, not a network-wide label-consensus guarantee.
- Record actual float64 serialization, including IDs, timestamps, counts,
  and states; reject malformed packets and require bitwise round trip.
  One directed communication round, with 16 KiB fragments and the existing
  128 B/node/frame modeled control. Publish raw and padded bytes separately.
  Both internal fusion stages are receiver computations, not hidden radio
  rounds. Keep fusion time separate from cached Local filtering time.
- Recompute position OSPA (p=2,c=12), count MAE, GOSPA localization/miss/
  false components from saved states, independently of MATLAB scoring.
  Pair complete episodes; report all seeds and 95% percentile bootstrap
  intervals (10,000 draws, seed 8301). Existing ER/no-age/MIL-S numbers are
  reused only after hashes are checked. Do not compare cached and new
  whole-pipeline wall times as a controlled runtime experiment.

## Gao FoV-MIL verification boundary

Audit the public author manuscript arXiv:1911.01083v1, especially Proposition
3, Section IV, and Section V-B/C. The 2022 TAES publisher abstract and
institutional record are available; the institution marks its full accepted
manuscript closed. Do not claim the 2022 implementation has been reproduced
from that abstract. Verify the existing known-label MIL-S specialization
against the public subspace equations and implement/test independent-label
matching for any new two-node replay. Any remaining publication-version
gap must remain explicit in the report and manuscript.

## Public-data extension

Select by input completeness, before tracking outcomes. First candidate:
the DMSTrack authors' released V2V4Real per-CAV no-fusion detections and
tracking labels, commit `d3b9949499c8e68ea33060873bd1cb95b6d4d323`.
Use all nine released evaluation sequences if poses and transformations
can be obtained. Inspect all frame counts and coordinate conventions first.
Do not create measurements from truth and describe them as real detections.
Do not infer ego motion from labeled target trajectories. Freeze the replay
model, association/birth settings, communication conditions and score domain
in a separate protocol before observing tracking outcomes. The two existing
event families remain mechanism case studies; the no-new-target case remains
a negative control. A real two-vehicle replay does not establish arbitrary
multi-robot network behavior or a full 3-D benchmark result.
