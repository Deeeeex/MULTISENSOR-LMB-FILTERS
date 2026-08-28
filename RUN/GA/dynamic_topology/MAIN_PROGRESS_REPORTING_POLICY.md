# Main progress reporting policy

The repository experiment ledger and the main progress document serve different
purposes.  Every completed candidate keeps a reproducible repository record,
but numerical results enter the main document only after passing the gate that
was frozen before the result was opened.

## What enters the main document

- the stable research problem, method rationale and design that define the
  current route;
- a deployable method result only after the registered M24 and X36 gates pass,
  including worst-sensor, worst-formation, rejoin and communication-budget
  constraints;
- a mechanism conclusion only when its own preregistered cross-scale mechanism
  gate passes and it materially changes the next method design; the text must
  identify it as an upper bound or diagnostic rather than a deployable method;
- validation on an additional scenario family only after its frozen tracking
  gate passes without retuning.

The main document reports conclusions and aggregate comparisons, not a diary of
version numbers.  It contains enough evidence to support each retained claim,
while detailed per-seed tables and implementation traces remain linked in the
repository.

## What remains repository-only

- every failed or below-gate candidate, even if one aggregate metric improves;
- M24-only screens when the claim requires joint M24/X36 evidence;
- debugging attempts, malformed or interrupted executions, threshold sweeps,
  ablations that do not change the accepted conclusion, and privileged teacher
  results that miss their registered mechanism gate;
- raw negative-result narratives for individual versions.

Several aligned failures may close a method family.  The main document may
state the resulting design constraint in one concise rationale, but it does not
promote the failed candidates or their numbers into a key result.

## Admission decision

Admission is claim-specific and fail-closed:

1. freeze the claim, comparator, scale set, metrics and thresholds;
2. run and save the candidate in the repository;
3. classify execution errors separately from scientific failures;
4. promote only a passed result at the scope supported by its gate;
5. otherwise keep the record repository-only and update the next design without
   adding a standalone section to the main document.

This policy applies prospectively to V145 and later experiments.  Earlier
below-gate V143 and V144 results remain repository-only.
