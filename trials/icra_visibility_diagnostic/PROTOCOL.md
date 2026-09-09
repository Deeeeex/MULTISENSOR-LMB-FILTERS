# Source visibility: post-outcome diagnostic

Start after miss-history archive `82ff5d95`. The miss-history rule failed its
frozen aggregate gate. Another source's current association confidence does
not distinguish most newly retained false outputs from the repaired target.

This diagnostic uses the same exposed cases: target ID 5 in `v2xt_0001`,
frames 53–122, and the four largest miss-history losses (`0006` and
`v2x_0002`, both links). It does not select a method or change the tracker,
labels, crop, scores, detections, or manuscript. Truth identifies the already
declared diagnostic subsets only.

1. Recover the existing detector boxes (center, height, width, length, yaw).
   Verify their prepared source-center coordinates against the frozen 2D
   measurements. These metadata were discarded by the current tracker input.
2. At each diagnosed negative source, use its saved **predicted mean**, before
   the current update. Ask whether the center ray from that source crosses an
   existing current detector footprint entirely before reaching the query.
   Exclude rectangles containing the source or the query. Use exact rectangle
   geometry with numerical tolerance only; no enlarged boxes, fitted angular
   margin, score threshold, or outcome-selected radius.
3. Report the complete declared subsets and missing local records. A positive
   ray intersection is a crude potential-occlusion cue; a negative result is
   not proof of visibility. A 2D footprint ignores height and unrecognized
   occluders. No causal performance claim follows without a new full recursion.
4. Inspect the existing V2X raw point clouds and all annotation classes at the
   focal true and false locations if detector geometry is insufficient. Keep
   observed signal separate from unproven explanations such as missing labels.

Bind every diagnostic to native input, source, and detector hashes. If the cue
does not separate the cases, record that result before considering a method.
