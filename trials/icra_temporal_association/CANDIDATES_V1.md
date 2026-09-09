# Current-observation association: two registered candidates

The original-nine instrumentation completed 18 native trajectories and
7,972 robot-frames with exact parity for every original output field except
runtime. Its raw LBP weights independently reconstruct the new observation
moments. The original pairing diagnostic is an annotation-based identity
diagnostic, not a calibrated estimate of error probability.

## Shared current-observation message

For each local component, normalize its LBP branch weights including the
missed-detection branch. Let a be the detection-association mass. Conditional
on a detection, compute its weighted center y, covariance S = I + the
weighted scatter of detection centers, maximum conditional weight p, and
entropy. These eight doubles are appended to the existing Gaussian packet:
416 bytes per Bernoulli plus the existing 32-byte packet header. Detector
noise is the unchanged local model Q = I. No annotation identity is sent.

A component supplies qualified current geometry when r*a*p >= 0.5. This is
a fixed majority-support criterion, not a calibrated confidence claim.
For a qualified pair compute d = (y_i-y_j)'(S_i+S_j)^-1(y_i-y_j).
Each d compares simultaneous measurements in their shared current ego
coordinates. It is invariant to a common planar rigid transformation.

## Candidate D: current observation

Use C = d/9.21034037197618 for a pair with qualified geometry from both
sources. Otherwise use the unchanged four-dimensional symmetric Gaussian
KL cost divided by 100. Retain each known equal-label pair unless its
qualified current C exceeds 1. Reopen those inconsistent known pairs and
solve the usual augmented assignment on all free endpoints, with private
unmatched costs 0.5 per endpoint. This preserves the original KL threshold
when current geometry is unavailable.

## Candidate T: three-frame observation consistency

Use the same procedure, but sum the available d values for the same two
source labels in the current frame and preceding two frames. The receiver
may use a historical pair only when that remote packet was delivered and
both components had qualified current geometry then. The current pair must
also be qualified. Divide the sum by the nominal 99% chi-square threshold
for 2*k dimensions: {9.21034037197618, 13.2767041359876, 16.8118938297709}
for k={1,2,3}. Repeated errors may be correlated; these constants specify
the algorithm and do not establish a calibrated false-rejection rate.
Store at most two previous received snapshots. No future or undelivered
remote observation enters the history.

## Identity conflict and abstention

An unmatched remote component whose original label collides with a local
label after reopening is omitted from this receiver's fusion inputs. If
the reopened local component has no replacement match, treat that remote
source as abstaining for this label, using the existing label-specific
weight transfer. A rejected identity correspondence must not masquerade as
a missing-target observation. Unrelated missing labels retain the original
FoV-censored absence rule. Unique output labels and original source keys
are checked explicitly; the original GCE density equation is unchanged.

## Evaluation and exit

Freeze both implementations and all input hashes before their first native
trajectory. First run original segment 0000 with unchanged GCE, D and T,
requiring baseline parity, packet round trips, independent assignment,
density, moment and output audits. Then run D and T on the remaining eight
original development segments, in both fixed link conditions.

Select the lowest two-condition sequence-macro OSPA mean, with exact ties
preferring D. Advance only if both link-condition OSPA means improve over
original GCE and the pooled 2 m wrong-pair rate does not increase. Report
missed common-identity opportunities and final adjacent-frame switches
alongside that rate. Preserve both candidates even if neither advances.
The already viewed V2X validation data and full V2V release remain exposed
assessments. Any additional recording must be frozen before its tracking
outcomes are read. Matching gains must ultimately be paired with the same
matching frontend on a conservative fusion baseline.
