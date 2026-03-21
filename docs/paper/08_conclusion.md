# Conclusion

## Conclusion Skeleton

Suggested structure:

1. Restate the problem
   Fixed-weight KLA fusion is brittle under time-varying communication and sensor quality.
2. Restate the method
   We proposed an adaptive, consistency-aware weight allocation strategy for distributed GA-LMB or KLA fusion.
3. Restate the key technical point
   Robust decoupled NIS is the critical design element.
4. Restate the main empirical message
   The method improves consensus quality and is more stable than plain NIS weighting.
5. State the limits
   Current evidence for `history` and `association ambiguity` is weak, and stronger baselines remain desirable.

## Discussion Points

- Why consensus improvement matters in distributed tracking
- Why consistency-aware weighting is safer than monotonic NIS rewards
- Why optional modules should remain secondary until stronger evidence is available

## Possible Future Work

- stronger baseline comparisons
- richer communication models
- larger-scale networks
- heterogeneous sensing modalities
- more principled ambiguity-aware weighting
