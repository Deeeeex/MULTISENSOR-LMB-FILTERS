# Missing-label censor diagnostic

This is a post-hoc diagnostic on all nine development and all 25 formerly
reserved fusion sequences. All outcomes are already seen. It does not run or
fit a new tracker and cannot establish a recursive or causal improvement.

Use both saved, matched-information M-No-age and M-ER trajectories in both
radio conditions. At each actual fusion, retain exactly their aligned label
unions, present-source probabilities, spatial means/densities, and inputs.
A participating missing-label censor is identified by its recorded existence
0.001, zero timestamp, and neutral age factor 1, consistent with the unchanged
runner and missing-label implementation. There is at most one such source in
each two-source label record. The sole-source spatial log normalizer is zero
mathematically; the existing canonical-Gaussian calculation has small floating
point cancellation residues, which are retained exactly in this diagnostic.

Compare four fixed-input extraction rules: retain all censors; omit only a
self-source censor; omit only a peer-source censor; omit either censor. When a
censor is omitted, the sole represented-source log odds and the unchanged saved
spatial log normalizer determine its probability. This
does not add a new label or change the source's spatial estimate. Reapply the
unchanged pruning and MAP-cardinality extraction and independently score the
full frame. On frames without fusion, use the original output unchanged.

Evaluate every sequence and all frames, using sequence-macro OSPA and GOSPA
missed/false squared costs, with paired descriptive sequence intervals. Report
adverse effects as well as improvements. No subset, threshold, or new fusion
rule is selected within this diagnostic. A follow-up rule, if warranted, must
have its own protocol, complete recursive replay, and controls.

The initial diagnostic stopped before producing results because it demanded
an absolute 1e-12 zero-normalizer tolerance; the first saved inputs contained
residues up to 2.24e-11. The revised diagnostic keeps the original normalizer
instead of replacing it with an exact zero, and records its maximum magnitude.
No tracker, input, original output, or fusion rule was changed.
