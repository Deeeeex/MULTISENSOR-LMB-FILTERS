# V229 beneficiary-only query-key recall

- State: `x36-formation-fov / seed 1301 / t=133`
- Query cap: `3 labels`
- Teacher recall: `0 / 2 = 0.0%`
- Selection inputs: beneficiary current local posteriors only; no remote inventory, truth or future outcome

| Row | Beneficiary | Source / teacher label | Ranks: existence / spatial / uncertainty | Selected | Support / mean r |
|--:|--:|:--|:--|:--|:--|
| 1 | F5 | S2 / `[1,4]` | `8 / 9 / 7` | `0` | `6 / 0.736` |
| 4 | F6 | S1 / `[25,20]` | `6 / 10 / 3` | `0` | `6 / 0.658` |

## Selected keys by formation

- F5: `[19,14], [19,16], [25,18]`
- F6: `[25,19], [31,21], [1,3]`

## Decision

The frozen three-mode beneficiary-only selector misses at least one audited teacher label and is not authorized as the V228 query-key rule. Preserve teacher headroom as a mechanism test and design a source-offer or query/offer hybrid control plane that can expose remote surprise evidence.

## Evidence boundary

This is a single opened X36 t=133 causal-discoverability diagnostic. Failure rejects this simple beneficiary-only three-mode selector for the current teacher rows; it does not prove that every beneficiary-only model must fail and is not tracking, closed-loop or generalization evidence.
