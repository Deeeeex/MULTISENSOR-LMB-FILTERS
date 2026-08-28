# V141 undercount-only cardinality augmentation

## Frozen decision

V140 failed because it matched the exact-reference cardinality in both
directions.  Its 25 effective M24 projections all had `k_R<k_H`, so the rule
deleted useful hybrid labels and reduced intervention gain from `+5.614%` to
`+0.765%`.  The existing X36 diagnostic has the opposite and much narrower
failure: the only six unhandled cells with `k_R>k_H` are exactly all six
severe negative cells, each with `k_R=k_H+1`.

V141 therefore applies a sign gate with no fitted threshold:

1. build the complete V139 hybrid output H and exact relay reference R;
2. compute their ordinary MAP cardinalities `k_H` and `k_R`;
3. when `k_R<=k_H`, return H without changing any label or state;
4. when `k_R>k_H`, keep the existing H output byte-for-byte and append the
   next highest-existence H labels until the output cardinality reaches
   `k_R`;
5. if H lacks enough eligible Bernoulli components, preserve H rather than
   replacing it by R;
6. apply the inherited V139 predictive fallback afterward.

Neither hidden posterior lineage is modified.  The gate reads no target truth,
future measurement, result mask, or tuned margin.

## Structural properties

For a represented LMB, the top `k_H` existence probabilities are a prefix of
the top `k_R` probabilities whenever `k_R>k_H`.  V141 consequently preserves
the original hybrid label set and only appends conditional-MAP labels.  The
implementation also preserves every existing extracted state and selects
components only for newly appended labels.

Let X be the truth set, H the original hybrid estimate and C the augmented
estimate.  If `|H|<|C|<=|X|`, and C retains all states in H, then OSPA cannot
increase: an optimal assignment for H remains feasible, each added estimate
can be assigned to one formerly unmatched truth at cost no greater than the
cutoff, while the cardinality penalty decreases by exactly one cutoff term.
This is a conditional metric property, not a claim that `k_R<=|X|` is known
online or universally guaranteed.

## Screen and claim boundary

V141 inherits the frozen V139/V140 action, seed, route, delivery realization,
predictive fallback, communication accounting and gates.  M24 must first be
identical to V139 and pass its registered gate.  Only then is X36 opened.

The exact relay reference and charged dual payload remain privileged teacher
state.  Even a dual-scale pass establishes only a mechanism target for a later
observable selector or compressed side channel; it is not yet a deployable
communication method and must not enter the main Lark document as a method
result.
