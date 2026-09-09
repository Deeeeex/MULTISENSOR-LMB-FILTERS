# Restore the original observation domain before interpreting association results

The V2 preflight failed exact original-GCE trajectory parity on 0001. The
first material difference was at frame 8, sensor 2, label (2, 200001): the
original run had pD = 0, whereas the new baseline had pD = 0.9. This is an
execution-environment error, not a candidate-method effect.

`checkObservationAssociation` adds the common directory at the beginning of
the MATLAB path. Running that check after installing `replay_quality`
therefore shadows the original replay observation-domain adapter. V1 calls
that check directly; V2 calls it through `checkPersistentAssociation`.
The adapter excludes the original rectangular-domain boundary and the
three-metre areas around both vehicles. The generic common implementation
does not apply those additional exclusions.

Preserve all original V1 and V2 runs, audits and selections as diagnostic
records. Their comparisons to the original baseline are invalid and must
not be used to select a method, report a benefit, or reject a method. The
instrumentation replay and fixed-original-input diagnostics are unaffected:
their runner does not call the path-changing association unit check.

The restoration reinstalls `replay_quality` after the unit checks and
asserts the resolved quality function and Hungarian implementation. No
candidate equation, threshold, persistence rule, split label, input, seed,
or fusion equation changes. One bounded runner supports the four already
specified candidates by calling their unchanged matcher implementations.

The independent auditor additionally reconstructs the observation-domain
gate from every retained local component's predicted state and the current
vehicle positions. It checks the executed pD and current-opportunity flag.
Exact original-GCE trajectory parity is mandatory on original segments
0001 and 0008 before the remaining seven segments run. Every accepted
association, causal history transition, split, actual message size and
Gaussian density remains independently checked. R and S must reproduce the
original trajectory on the no-persistent-conflict segment 0008.

Rerun D, T, R and S on all nine original development segments and both
link conditions. Reuse only the unchanged original GCE trajectories outside
the two baseline preflights. Advance only a candidate with lower
sequence-macro OSPA in both link conditions and a nonincreasing pooled
2 m wrong-pair rate. Among eligible candidates choose the lowest
two-condition mean; exact ties follow D, T, R, S order. Report all four
restored candidates. The candidate definitions remain those in
`CANDIDATES_V1.md` and `CANDIDATES_V2.md`; this repair does not retune them.

The previously frozen additional V2X-Real test cohort remains unread.
An advancing candidate still requires the complete exposed-release
assessment, a conservative fusion control with the identical association
frontend, and the additional test cohort before paper promotion.
