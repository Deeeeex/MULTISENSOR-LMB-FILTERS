# V231 budget-adaptive source-offer recall

- State: `x36-formation-fov / seed 1301 / t=133`
- Sources / offers per source: `9 / 12`
- Mode breadth: `6 existence + 6 spatial`
- Control charge: `2880 B`
- Payload charges: `1456, 1456 B`
- Certified net savings: `6064, 6064 B`
- Teacher recall: `2 / 2 = 100.0%`
- Actual runtime received cache used: `0` (same-state proxy only)

| Row | Beneficiary | Source / label | Rank E / spatial | Selected | Single-cache recall |
|--:|--:|:--|:--|:--|:--|
| 1 | F5 | S2 / `[1,4]` | `5 / 8` | `1` | `6/6 = 100.0%` |
| 4 | F6 | S1 / `[25,20]` | `17 / 6` | `1` | `6/6 = 100.0%` |

## Decision

The byte-derived proposal breadth recovers every opened teacher row and preserves positive certified saving. Persist and test the actual source-local runtime cache next; do not train or claim tracking gain yet.

## Evidence boundary

V231 replaces the failed arbitrary top-two source cap with a breadth computed from causal byte credit. Same-state recall shows only whether the proposal layer can retain the opened teacher rows while preserving positive communication saving. It does not validate the runtime cache, the beneficiary coordinator, tracking gain, or generalization.
