# Admission revision following the latest ICRA assessment

Start from `cb67ee81`, retaining the completed GCE, control, motion, pD and
new-recording trajectories. The full-public-coverage background run also
continues with its original frozen sources. New algorithms live in this
separate directory and receive new arm names.

## Questions and first candidates

1. Decompose the curvature-guard reversal on official validation 0000 into
   localization, missed-target and false-target costs. Compare rejected
   positive and negative existence increments, admission strengths, and
   minimum eigenvalues of the reconstructed precision change. At fixed GCE
   inputs, restore rejected scalar evidence while keeping the saved spatial
   posterior; compute the resulting set-extraction and error changes.
   Run the same diagnostic on all nine original development sequences.
2. Test factorized admission: retain the original spatial guard and
   integrability fallback, while allowing separately selected scalar
   increments from a source whose spatial increment is rejected. The first
   three candidates restore all, positive-only, or negative-only discarded
   scalar evidence. Each runs its own complete recursion. No new scalar
   strength or pD parameter is introduced in these three candidates.
   The completed fixed-input diagnostic shows a false-target/missed-target
   tradeoff for negative restoration. Before any new recursive outcome,
   add a fourth candidate that restores rejected negative scalar evidence
   only when both current participating sources have negative increments.
   Record the unconditional candidates as the direct mechanism controls.
3. Extend the fixed-strength reference with eta in {0, 0.05, 0.1, 0.125}.
   Combine these with the previous {0.25, 0.5, 1} grid. Select by the mean
   of reliable/intermittent sequence-macro OSPA on the same nine development
   sequences, breaking exact ties toward the smaller strength. Keep every
   candidate result.

## Development and evaluation

Use the nine development sequences for candidate comparison and the
409-frame validation recording for the reviewer-requested mechanism
diagnostic. Algorithm changes motivated by this diagnostic make it a
development case for this revision. Freeze candidate definitions and
selection before evaluating the chosen version on the complete remaining
release. Summarize by original recording as well as by sequence.

Acquire a separate public real recording for evaluation after the candidate
is frozen. Inspect archive identities, input compatibility and poses during
development; do not inspect its tracking outcomes or choose recordings by
their tracking score. Reuse the detector, calibration and primary tracking
settings wherever the source format permits, recording any input change.

The objective is a verified accuracy improvement, supported by a specific
explanation of what changed. Continue with documented, separately frozen
revisions if the first candidates fail. A favorable fixed-input diagnostic
alone does not select a final method.

## Verification and paper

Check scalar/spatial separation, sign selection, absence/current-opportunity
conditions, aggregate fallback, original-arm parity, and fixed-strength
endpoints with numerical fixtures. Verify complete native exits, inputs,
packet accounting, distributions, extraction and independent error scoring.
Keep all attempted versions and results; never overwrite existing evidence.

Use the selected complete results to revise the manuscript's method,
experiments and abstract. Explain the problem, solution, effect size and
experimental conditions directly. Keep the approved figures editable and
the seven body pages plus one acknowledgment/reference page. Regenerate and
visually check the PDF and portable source package before final delivery.
