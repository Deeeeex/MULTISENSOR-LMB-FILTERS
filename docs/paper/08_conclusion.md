# Conclusion

## Conclusion Skeleton

Suggested structure:

1. Restate the problem
   Fixed-weight KLA fusion is brittle under time-varying communication and sensor quality.
2. Restate the method
   We proposed an adaptive weight allocation strategy for distributed GA-LMB or KLA fusion using covariance, realized link quality, and existence confidence.
3. Restate the key technical point
   Existence confidence is the critical additional design element beyond covariance and link quality.
4. Restate the main empirical message
   The method improves consensus quality under tiered heterogeneous packet loss, and existence confidence further improves all three consensus metrics beyond the `covariance + link quality` baseline.
5. State the limits
   Current evidence for `freshness`, `history`, `association ambiguity`, and `cardinality consensus` is weak or negative, and stronger Monte Carlo baselines remain desirable.

## Discussion Points

- Why consensus improvement matters in distributed tracking
- Why existence-confidence captures information missed by covariance and link quality
- Why consistency-aware weighting is safer than monotonic NIS rewards
- Why optional modules should remain secondary until stronger evidence is available

## Possible Future Work

- stronger baseline comparisons
- richer communication models
- larger-scale networks
- heterogeneous sensing modalities
- joint use of existence confidence and consistency penalties
- more principled ambiguity-aware weighting
