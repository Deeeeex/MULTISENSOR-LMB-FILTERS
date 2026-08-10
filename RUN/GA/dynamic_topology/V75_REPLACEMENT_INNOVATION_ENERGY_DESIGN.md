# V75 replacement innovation-energy design

V74 shows that self-posterior Bayes risk misses confident spatial bias and
that zero-tolerance whole-network disagreement rejects every local
information injection.  V75 therefore adds one direct, slot-local spatial
risk instead of relaxing the V74 thresholds.

For each exact receiver--incumbent--candidate replacement and each label
supported by all three local LMB posteriors, V75 computes

`q = (mu_candidate-mu_incumbent)' * (P_incumbent+P_candidate)^(-1) *
(mu_candidate-mu_incumbent)`.

The label weight is the receiver existence probability times the smaller of
the two sender existence probabilities.  A slot uses the weighted mean label
energy and a formation uses the maximum slot energy.  The hard risk uses the
conditional energy when the candidate message arrives; current link
reliability is reported separately and cannot hide a rare but harmful
replacement.

The frozen development limit is `5.991464547`, the conventional 95% chi-square
reference for a two-dimensional normalized innovation.  The local LMB
posteriors are correlated and can be multimodal, so this is a scale reference,
not a calibrated p-value.  More importantly, the limit was selected after
inspecting the two opened anchors.  The current run is therefore a feasibility
falsification only; the formula and limit must remain fixed for later new
scene--seed evaluation.

V75 evaluates both exact route generations.  The historical V71 triples are
the ones executed in V72 and may be compared retrospectively with the opened
formation-level first-step directions.  The V73 triples changed one candidate
sender at each scale and remain unexecuted; their energy classifications are
prospective diagnostics, not tracking labels.

The eventual direct gate is conjunctive but keeps distinct semantics:

1. nonnegative affected-formation posterior Bayes objective;
2. no reference-supported label decision down-crossing or expected-cardinality
   loss;
3. maximum conditional replacement innovation energy no larger than
   `5.991464547`.

Whole-network disagreement is removed from the direct gate.  It becomes a
separate multi-round recovery cost.  Existing predictive-measurement reward
is not reused because it needs the next measurement and has already failed its
registered alignment gates.

V75 reads no truth or future measurement, executes no route, and trains no
model.  If the formal Octave result does not keep M24 formation 3 and X36
formation 4 below the fixed energy limit while placing X36 formation 5 above
it, the RIE branch is rejected without threshold tuning.
