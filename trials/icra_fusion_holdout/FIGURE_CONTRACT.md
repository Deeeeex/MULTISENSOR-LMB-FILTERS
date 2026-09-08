# Complete-cohort result figure contract

Scientific role: assess the fixed evidence-ceiling rule against matched-
information No-age and ER using all 25 reserved fusion sequences. The figure
must permit either a favorable or an adverse conclusion; no plot selection,
unit removal, truncation, axis break or threshold change follows the scores.

Archetype: quantitative grid with the paired OSPA panel as the primary
evidence, followed by a matched false/missed-cost panel showing the tradeoff.
The four rows are No-age and ER comparisons under reliable and intermittent
links. Show all 25 sequence differences and their macro mean with the exact
registered 95% sequence bootstrap interval. Keep the same row order across
panels. Per-sequence marker displacement depends only on sequence ID order.

Backend: the existing Python/matplotlib workflow, used for drawing, preview,
export and QA. Fixed 183 mm by 90 mm vector canvas, 7 pt body text, embedded
TrueType PDF fonts and editable SVG text. Export PDF, SVG and a 600 dpi PNG.
Write every plotted point and interval to CSV with source sequence IDs and
hashes of the complete independently audited summary and method registration.

The interval unit is a complete sequence, n=25, all 5601 frames. Use the
already registered 10000 resamples and seed 8301; no multiplicity adjustment
or new statistical calculation is introduced by the plotting script.
Calibration uses the nine development sequences only. The detector was
trained on the held-out-for-fusion train split; related routes remain possible.
No microscopy or image transformations are involved. Negative OSPA/cost
differences favor the candidate. Neither positive intervals nor adverse
sequences may be cropped from the axes. The communication and common-target
localization comparisons are reported in tables, with their actual support.

Before delivery, check exported live text, all drawn text within the canvas,
source point means against the audit, final physical size, no Type 3 fonts,
and visual readability at publication scale. The paper's page limit is not
an optimization constraint in this method-development request.
