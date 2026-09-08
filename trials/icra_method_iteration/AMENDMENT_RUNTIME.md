# Runtime-only amendment, 2026-09-08

The detailed replay reproduced the original ER states bitwise on all 2,412
node-frames in development sequences 0000--0003, but became extremely slow
on the dense 0004 sequence. A bounded MATLAB profile attributes 21.498 of
23.130 seconds to Hungarian step 4, which scans matrix entries one by one
to find the first uncovered zero. Fusion diagnostics account for under half
a second. The two unfinished 0004 executions were interrupted, retaining
all completed earlier sequence files and logs; they are not complete-cohort
evidence. A separate original-runner diagnostic also remains outside the
reported cohort.

Use a trial-local copy of the existing Hungarian implementation with only
that scan vectorized. Search a transposed logical zero/coverage mask so that
the first found zero has exactly the original row-major order. Keep every
cost, assignment step, tie break, dummy node, label rule and threshold
unchanged. Do not edit the core or pinned author source. Check exact matching
matrices against the original on empty, rectangular, tied, finite/Inf and
augmented dummy-slot problems before use. Then require the accelerated
instrumented ER to reproduce all original saved state outputs, and the
completed slow IR/CR outputs to match the accelerated versions on their
common sequences. A mismatch is a stop condition, not a numerical tolerance
for selecting a more favorable method.

Write accelerated results in separate directories and record the exact
runtime source hash. Rerun all nine development sequences for both candidates
and retain their full per-label diagnostics. The method rules and original
data/parameters remain frozen. For the reserved real-data cohort use the
same assignment implementation for every arm. This acceleration is an
implementation change, not a tracking-method contribution; do not infer a
whole-pipeline runtime advantage from the historical baselines.

The acceleration preflight exposed a coverage-vector orientation issue:
the original step 3 returns a row vector while step 2 initializes a column
vector. Normalize both vector orientations in the logical mask. The first
unit-check attempt was stopped before any accelerated tracking output.
Rerun the complete matching-parity checks after this correction.
