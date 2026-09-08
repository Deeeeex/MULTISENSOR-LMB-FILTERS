# Remaining fusion-selection cohort: all 25 unused train sequence IDs

Cohort registered 2026-09-08 before new tracking outcomes. The earlier
development set is all nine released validation sequences. The earlier
fusion transfer used train IDs 0000, 0005, 0010, 0015, 0020, 0025 and 0030;
0000 was an exact ground-truth overlap control. Their outcomes are now seen.

Use every other train sequence: 0001, 0002, 0003, 0004, 0006, 0007, 0008,
0009, 0011, 0012, 0013, 0014, 0016, 0017, 0018, 0019, 0021, 0022, 0023,
0024, 0026, 0027, 0028, 0029 and 0031. This is 5601 frames; no replacement,
frame window, condition or sequence may be chosen from its tracking score.
The previous exhaustive geometry audit found no exact cross-split ground-
truth sequence duplicate among these IDs. This does not prove independent
routes. The released detector was trained on this train split: reserve it
only for fusion-method selection, never call it a detector benchmark test.

Preparation may inspect metadata and reproduce the exact established 2D
coordinate conversion, crop, poses and per-sensor detection ordering. Fetch
only the 11202 small transform arrays from the pinned public train ZIP,
checking ZIP CRCs and file hashes. Do not download the 19.9 GB features.
Keep raw released scores as optional additional inputs; calibration must be
fixed using previously seen sequences before any cohort tracking output.

Methods and paired comparisons will be registered in a separate immutable
method freeze after the complete development and fair mark-only comparison.
No tracking result from this cohort is permitted before that freeze. Both
existing reliable and intermittent radio conditions will be included.
