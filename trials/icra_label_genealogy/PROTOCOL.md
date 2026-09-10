# Population balance and birth ancestry in nominal recursive failure

Continue the original GCE versus No-age cause diagnosis. The preceding
curvature-conflict screen is closed after all four gates failed. No new
admission rule or performance screen is introduced here.

Use all six nominal `v2xt_0001` trajectories in the completed nominal-origin
trace: GCE, Guarded Scalar and No-age, reliable and intermittent links,
all 240 frames and both robots. This is an already exposed diagnostic case.
Keep all data, local filtering, source matching, fusion and extraction
unchanged. The existing GT 5 / 2 m / frames 53--122 diagnostic is retained
only for describing the established failure; all population balances and
ancestry calculations cover every label and every frame.

First verify the actual recursive bookkeeping. At each robot/frame, the
prediction consists exactly of the preceding retained posterior labels
plus births from every previous local detection. Birth labels, existence
0.01, zero velocity, measurement-centred mean and diagonal covariance
(16,16,225,225) follow the frozen runner. Existing objects have survival
0.99 and the original constant-velocity prediction. Reconstruct and check
all predicted existence/means, all available predicted covariances, and
the complete local pruning decision at existence >0.001.

At a received fusion, join each output label to its exact local and remote
pre-alignment source labels. Count local-only, matched and remote-only
outputs; check that no source label is used twice at a receiver. Check
both fusion pruning and the population identity

    retained_now - retained_previous
      = births - local_pruning - local_labels_lost_at_fusion
        + retained_remote_only_labels.

No reception must be exact local-posterior continuation. Classify each
local survival and deletion by birth versus inherited label; compare the
actual birth rows across methods without assuming inherited inputs match.
Report full-run and fixed-window totals, cohort ages and first count
differences. Retain counts of current-frame labels and labels born within
the first three frames, which contain the already established first
divergences. Label age is not the physical object's age.

Second propagate a diagnostic ancestry set. Each original birth is a leaf
identified by (birth frame, source sensor, previous-detection index). Local
prediction/update inherits its label's previous set; a new birth starts a
singleton. Fused output ancestry is the union of the actual source sets
recorded in `fusionSourceRecords`. Pruning discards that node. This records
birth-hypothesis ancestry only: current measurements are not new leaves,
and shared ancestry is not by itself proof of double-counted likelihood,
the same real target, or an implementation error.

For every retained pool report unique birth roots, ancestry memberships,
roots represented in multiple labels, labels involved in such overlap,
and pairs of labels with shared ancestry. Report the same quantities in
the fixed target's 2 m neighbourhood and preserve exact label/root sets.
Record how ancestry overlap first appears and whether it arose in local
continuation or a particular received matching. Do not infer a full
recursive causal explanation from these counts alone.

Freeze source, roster, thresholds and the complete reporting scope before
producing the census. The producer uses integer bitsets; an independent
verifier reconstructs ancestry with ordinary birth-ID sets from the native
records, and independently checks every population row, preserved root
set and CSV/JSON aggregate. Preserve all failures without modifying old
native runs. Any later intervention requires a separate frozen protocol;
this trace cannot promote a method or constitute independent validation.
