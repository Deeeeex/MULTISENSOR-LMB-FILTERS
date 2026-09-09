# Compatibility and missed-evidence development screen

The first two candidate families have completed all nine development
sequences. Scalar restoration did not improve the registered aggregate.
Contracting-space restoration reduced mean OSPA by only 0.002099089 m,
with opposite changes across the two radio conditions. Its frozen full-corpus
and V2X evaluations are running, but their tracking scores have not been
inspected when this third family is defined. Use only the original nine
sequences and the previously diagnosed 409-frame recording for development.

The next question is whether current increments should be admitted at full
local support when the two spatial posteriors or current signs disagree.
The Gaussian overlap integral of the conservative spatial pool is already
available at the receiver. With two equal spatial weights it is the
Bhattacharyya coefficient, in [0,1]. Use it directly, without a fitted scale,
to modulate current exponents. Retain the original full-matrix curvature
guard, old history weights, local tracker, pD=0.9, marks and packet schema.

Define five fixed-input candidates:

1. `agreement_all`: multiply every raw current exponent by the overlap.
2. `agreement_positive`: multiply positive current exponents by overlap;
   retain the original negative exponents.
3. `agreement_conflict`: multiply current exponents by overlap only if
   participating current log-odds increments have opposing signs.
4. `positive_consensus`: admit positive current exponents only when all
   participating increments are positive; retain original negative exponents.
5. `positive_only`: admit positive current exponents and set additional
   negative current exponents to zero. The local missed-detection update is
   still present in the conservative posterior pool. This directly tests
   whether repeated nominal missed-detection evidence is harmful.

Each modified exponent applies to both the scalar and spatial part of its
current Bernoulli ratio, followed by the original guard and normalization.
No candidate uses object truth, a fitted threshold, additional communication,
or changes to the local recursion. Compare all five on the old fixed inputs.
Advance the best two by mean reliable/intermittent sequence-macro OSPA on
the nine development sequences, plus `positive_only` as the direct missed-
evidence control if it is not already selected. Exact ties use the order
above. Run every advanced candidate's full recursion on all nine sequences
and retain every outcome. This fixed-input screen is a computational filter;
it does not establish recursive accuracy.

Keep the other families and their frozen comparisons intact. Any new final
choice must use complete audited development recursions and be recorded
before inspecting external candidate tracking scores. Record precisely which
external cohort was unseen at each choice; do not claim that repeated use of
an already inspected cohort is a fresh generalization test.
