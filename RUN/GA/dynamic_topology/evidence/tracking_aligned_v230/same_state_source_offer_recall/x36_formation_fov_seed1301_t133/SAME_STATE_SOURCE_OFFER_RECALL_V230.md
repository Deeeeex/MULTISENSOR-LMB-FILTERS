# V230 same-state source-offer ranking screen

- State: `x36-formation-fov / seed 1301 / t=133`
- Offer cap: `2 labels per source`
- Full-cache teacher recall: `0 / 2 = 0.0%`
- Actual runtime received cache used: `0` (same-state full-cache proxy only)

| Row | Beneficiary | Source / teacher label | Selected offers | Teacher rank E / spatial | Source r / reference r | Single-cache recall |
|--:|--:|:--|:--|:--|:--|:--|
| 1 | F5 | S2 / `[1,4]` | `[31,23], [19,16]` | `5 / 8` | `0.034 / 0.736` | `0/6 = 0.0%` |
| 4 | F6 | S1 / `[25,20]` | `[13,9], [25,19]` | `17 / 6` | `0.504 / 0.658` | `0/6 = 0.0%` |

## Decision

The ranker misses at least one teacher row even with a perfectly fresh full cache. Do not integrate this score into the recursive filter; redesign the source-side statistic first.

## Evidence boundary

V230 first evaluates source-relative ranking with current beneficiary posteriors standing in for a perfectly fresh full cache. A pass shows only that the score can express the opened teacher rows. It does not show that the runtime source owns those cache entries, that stale entries preserve recall, or that a selected offer improves tracking.
