# Origin of recursive GCE/Guarded Scalar divergence

Start from `fdd27cd9`, after the joint-admission fixed-input screen failed.
The exposed range-detection case previously showed 136 nearby Guarded
Scalar hypotheses versus one GCE hypothesis at frame 53. Substituting GCE
components at those already different frame-53 inputs did not repair the
target. The earliest divergence remains unlocated.

This is a descriptive trace of the already designated `v2xt_0001` case,
not a new candidate screen. Use all twelve saved preflight trajectories:
nominal and range detection models, reliable and intermittent links, and
GCE, Guarded Scalar and No-age. Include every frame. The range model remains
a failed shared-model candidate and cannot be promoted from this case.
No truth, packet, sensor setting, recursive output or score is changed.

For the already designated truth ID 5, report every robot/frame with the
truth present: nearby predicted/local/post-fusion hypotheses within 2 m,
active existence threshold 0.001, maximum and total nearby existence,
the strongest label, native extracted detection, and output count. Nearby
labels are competing hypotheses, not automatically false extracted objects.
Also report the nearest current detection, its original likelihood mark,
and the label distribution for that detection from r_plus*W. Normalize
that column only to describe its entropy and largest share; preserve its
unscaled total, and do not call LBP an exact assignment distribution.

For GCE versus Guarded Scalar within each sensor/link condition, scan in
time order and report the first difference in predicted mean/existence,
retained local distributions and post-fusion distributions. Compare labels
before numeric values. Distinguish exact equality from tolerances 1e-9 for
existence and 1e-7 for means/covariances; these suppress arithmetic-scale
noise only and are frozen before this trace. A first divergence does not
prove that one event causes a later target failure.

At the first different fused distribution, retain all source records and
show whether both source local inputs, matching and retained exponents
were still equal. Decompose the existence difference into the old spatial
integral, admitted scalar increments and new spatial integral. Independently
reconstruct those quantities from local Gaussian moments. Do not infer a
mechanism merely from an aggregate curve or a changed output count.

This trace can motivate a separately registered recursive intervention,
but does not perform one. A later causal experiment must identify its exact
intervention time and components before its native outcomes and preserve
the full unchanged control trajectory. The trace alone cannot establish
an accuracy improvement, a new method, or a GCE advantage over No-age.
