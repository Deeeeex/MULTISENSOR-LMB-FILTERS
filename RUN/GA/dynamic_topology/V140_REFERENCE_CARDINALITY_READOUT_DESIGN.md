# V140 reference-cardinality-constrained hybrid readout

## Why another output rule is necessary

V139's X36 loss is not caused by a broad failure of the working spatial
posterior.  At every severe cell, W has a better or nearly identical OSPA
localization component than the exact reference R, but its extracted set has
18 objects while R has 19.  Falling back to the whole R output destroys the
positive spatial gains at many other cells.  V140 therefore constrains only
the discrete set size and retains the hybrid spatial posterior.

## Frozen readout

For each protected receiver and current page:

1. construct the V139 hybrid LMB readout H using the frozen per-label W/R
   divergence comparison;
2. compute the exact-reference MAP cardinality `k_R` from R using the same
   pruning and Poisson-binomial cardinality calculation as the ordinary
   extractor;
3. extract exactly `k_R` labels from H, taking the labels with the largest
   existence probabilities and using H's existing mixture-component
   selection for their states;
4. if H contains fewer than `k_R` eligible Bernoulli components, use the
   exact R output for that page rather than silently returning a smaller set;
5. apply V139's frozen output-only predictive fallback afterward.  This can
   still replace the page by R, but neither operation mutates hidden W or R.

The method has no tunable threshold beyond the inherited LMB pruning rule and
no new score margin.  It uses neither target truth nor future measurements.

## Two exact properties

Let the unknown truth set be X, the exact-reference output be R and the V140
output be C.  V140 enforces `|C| = |R|`.  In OSPA of order p and cutoff c, the
cardinality contribution is

```text
c^p | |X| - |Y| | / max(|X|, |Y|).
```

Consequently C and R have exactly the same OSPA cardinality contribution and
normalization for every possible truth cardinality.  Relative regression can
then arise only from the localization term.  This is a relative safety
property, not a claim that either output is the Bayes-optimal MMOSPA estimate.

The fixed-cardinality label selection is also exact for the represented LMB.
For a label set I of size k, its LMB mass is proportional to

```text
product over labels in I of r_l / (1 - r_l).
```

Since the odds are monotone in `r_l`, selecting the k largest existence
probabilities maximizes the LMB label-set probability conditional on
cardinality k.  V140 is therefore a conditional-MAP projection, not a new
approximation to the filtering density.

## Screen and claim boundary

V140 inherits V139's fixed action, seed, route, delivery realization, KLD
label readout, predictive fallback and gates.  M24 is screened first; a pass
authorizes the paired X36 screen.  Both scales must achieve at least `+5%`
intervention gain, nonnegative full and mature windows, minimum formation gain
of at least zero, minimum sensor gain no lower than `-0.01%`, and exact
whole-formation reentry.

V140 still consumes the charged W+R mechanism payload.  Even a dual-scale
pass establishes only that spatially useful W output can be made
cardinality-safe.  It does not authorize a communication-efficiency claim,
main-document result, learned online policy or richer-geometry validation.
Those steps require a budget-feasible reference/cardinality representation
and a frozen online action selector.
