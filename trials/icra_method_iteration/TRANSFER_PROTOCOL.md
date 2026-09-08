# Reserved transfer cohort (before tracking outcomes)

Select every fifth sequence by ascending ID from the 32 released V2V4Real
`train` sequences in the pinned DMSTrack source: 0000, 0005, 0010, 0015,
0020, 0025, 0030. Selection uses IDs only, before any tracking output or
metric inspection. Use complete sequences: 147, 217, 119, 145, 275, 450,
151 frames, totaling 1504 paired frames. Other sequences are not part of
this bounded check. Fetch only the two sensors' coordinate transform arrays
from the public `train_no_fusion_keep_all.zip`, not the 19.9 GB features.

The released detector was trained on this original train split. These are
reserved algorithm-transfer inputs, not a new detector test benchmark, and
their driving routes may be related. Keep this distinction in all reports.

Apply the identical conversion, 40 m geometry, 3 m platform exclusion,
relative-ego CV approximation, likelihood parameters, independent births,
Gaussian label assignment, two radio conditions, and scoring definitions
from the frozen earlier nine-sequence replay. Use seed 8301+sequence ID for
radio draws. No frame, object or sequence exclusion from observed outcomes.

Freeze the chosen method source and all cohort input hashes before running
any tracker on this cohort. Evaluate Local, no-age, original ER, the chosen
new rule, MIL-AM and TC-OSPA2 windows 5 and 10. Report paired sequence
differences and all adverse outcomes. Do not revise the chosen method using
these outputs and then continue to call the same cohort held out.

## Pre-tracking duplicate-data amendment

Before running any tracker on these inputs, an exact source-data audit found
that train sequence 0000's entire label file is byte-identical to val 0000.
All 147 frames also have identical sorted 3-D box dimensions, centers and yaw
when IDs are ignored. Detector files differ by one row, which does not make
this an unseen driving sequence. An exhaustive geometry-fingerprint check
of all 32 train sequences against all nine val sequences found no other
exact frame matches at five decimal places. This does not prove route
independence. Store the audit and original file hashes with the inputs.

Keep all seven originally selected sequences in the outputs. Mark 0000 as
an overlap control, excluded from the primary reserved-cohort aggregate.
Primary reserved IDs are 0005, 0010, 0015, 0020, 0025, 0030: six sequences,
1357 paired frames. This exclusion is based on data identity before tracking
outcomes. Do not replace it with a favorable sequence or hide its results.

## Candidate-set amendment before reserved tracking

The complete CR synthetic audit exposed a discovery/false-cost tradeoff.
The separately registered CGR rule tests current two-hit confirmation as an
exception to the CR cap. Fix **both** CR and CGR in the reserved comparison,
along with Local, no-age, original ER, MIL-AM and TC windows 5 and 10: eight
arms in total. Freeze each complete development summary and exact source
before any reserved tracking. Report both candidates and all six reserved
sequence results. No further method change may use this cohort's outcomes
while retaining the reserved-validation interpretation.
