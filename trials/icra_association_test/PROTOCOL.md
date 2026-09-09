# Additional frozen V2X-Real test cohort

This directory reuses the previously checked V2X-Real input adapter and the
unchanged released V2V4Real detector. The acquisition cohort is exactly the
14 paired-mobile segments and 2,172 frame pairs registered in
`../icra_temporal_association/PROSPECTIVE_TEST_COHORT.json` before any of
their raw inputs were inspected. The cohort contains related segments from
five collection dates. It is not 14 independent recordings.

Acquisition is conditional on a Q, N or QN candidate passing the complete
nine-segment development gate in `CANDIDATES_V3.md`. The selected candidate
and its exact source hashes are frozen before acquisition. Do not change
the selected rule after any additional test output becomes available.

Keep the original detector checkpoint, class mapping, coordinate transforms,
point-cloud intensity handling, calibration, positive prior, birth/update
model, observation domain, current-ego CV, detection probability, extraction
and radio-draw construction. Detector inference uses only point clouds and
platform poses. Annotated object IDs and boxes enter the scoring conversion
after detection. Verify every raw file by archive CRC and SHA-256, check
exact cloud hashes against the exposed validation cohort, and audit every
saved tracking input against the frozen detector outputs and independent
pose/label conversion.

Compare original GCE and Guarded Scalar with the selected association rule
under both fusion equations. Include the original No-age KLA reference.
Every method runs its own full recursion on the same measurements and link
draws. Record added packet fields and actual wire cost. No training,
recalibration, scene filtering, parameter search or test-based method
selection is allowed.

Report every segment, both link conditions, OSPA, GOSPA decomposition,
cardinality error, actual communication, and the predeclared offline
association diagnostics where the required original-label records exist.
Report sequence, frame and collection-date aggregation separately, with
paired resampling at the collection-date level. A claimed transfer benefit
requires lower OSPA than original GCE in both link conditions; the identical
association frontend under Guarded Scalar is the fusion-attribution control.
Keep unfavorable results and do not promote a failed transfer as a general
improvement. This is the frozen two-dimensional replay evaluation, not the
dataset's official three-dimensional detection benchmark.

As a secondary mechanism breakdown, report frames with overlapping sensor
disks separately from the remaining frames. Define disk overlap solely by
the known platform positions: two 40 m sensor disks overlap when the planar
platform separation is less than 80 m. Apply this fixed rule to every method
and both link conditions, without inspecting detections, annotation counts
or tracking errors. The complete 14-segment result remains the primary
comparison; this breakdown cannot select or remove a segment.
