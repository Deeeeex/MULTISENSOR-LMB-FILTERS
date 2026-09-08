# Released split names and the present experiment's roles

The nine-sequence, 1993-frame directory is named `val` in the released
DMSTrack tracking interface. This directory name should not be equated with
the original dataset's validation role: the pinned author code calls the
same cumulative lengths the **test set**, and the author README describes
its headline experiment as evaluation on the testing split.

Evidence in the pinned author checkout (`d3b9949499c8e68ea33060873bd1cb95b6d4d323`):

- [main_dkf.py, test-length comment](https://github.com/eddyhkchiu/DMSTrack/blob/d3b9949499c8e68ea33060873bd1cb95b6d4d323/DMSTrack/main_dkf.py#L72)
  lists cumulative lengths ending at 1993.
- [main_dkf.py, interface split configuration](https://github.com/eddyhkchiu/DMSTrack/blob/d3b9949499c8e68ea33060873bd1cb95b6d4d323/DMSTrack/main_dkf.py#L829)
  assigns these same nine sequence IDs and lengths to `val`.
- [V2V4Real README, testing configuration](https://github.com/eddyhkchiu/DMSTrack/blob/d3b9949499c8e68ea33060873bd1cb95b6d4d323/V2V4Real/README.md#L143)
  instructs setting the validation-directory option to the testing data path.

In **our** experiment, these nine released sequences have already been used
for fusion-method development and score calibration. The 25 additional
sequences are all remaining author-interface `train` sequences after the
seven previously inspected controls are excluded. Their 5601 frames were
reserved for fusion-method selection outcomes, while the released detector
was trained on that split. These are experimental roles, not a new official
train/validation/test split or an independent detector test.

Earlier shorthand such as "nine validation sequences" refers to the author
interface directory name. Future reports should instead say "nine development
sequences from the released tracking outputs" and state the detector-training
limitation for the additional cohort. This clarification changes no sequence,
input, fitted coefficient, method, comparison, or metric in the fixed runs.
