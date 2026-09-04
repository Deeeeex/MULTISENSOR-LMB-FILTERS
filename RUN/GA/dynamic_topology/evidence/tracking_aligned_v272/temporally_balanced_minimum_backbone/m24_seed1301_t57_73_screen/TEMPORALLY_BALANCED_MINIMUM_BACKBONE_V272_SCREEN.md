# V272 temporally balanced minimum backbone

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Generation commit: `fa16508f9aa42c0a6bde25b1a8dc9b539e565f91`
- Continuation: `t=57--73` from the V259 V242 state
- Candidate route-change fraction: `1.000`
- Route-schedule metadata bytes included in payload accounting: `0`
- Mechanism / full-M24 authorization: `0 / 0`

| Arm | E-OSPA | RMSE | Cardinality error | Consistency | F4 event E / RMSE | Window bytes | Spliced static saving |
|:--|--:|--:|--:|--:|:--|--:|--:|
| V242 minimum backbone | 118.651 | 16.844 | 9.941 | 130.254 | 123.683 / 26.636 | 2991480 | 10.041% |
| V272 temporally balanced minimum backbone | 119.712 | 16.173 | 10.142 | 131.888 | 124.847 / 26.476 | 3045744 | 9.908% |

## Gains over V242

| Scope | E-OSPA | RMSE |
|:--|--:|--:|
| Network | `-0.895%` | `+3.984%` |
| F4 event | `-0.941%` | `+0.598%` |
| Weakest formation | `-2.357%` | `-0.729%` |

- Consistency gain: `-1.255%`
- Window byte change vs V242: `-1.814%`
- Cardinality-error change vs V242: `+0.201`
- Short-horizon gate: `0`
- Next method decision: `close-or-revise-temporal-balancing-without-outcome-tuning`

## Decision

The temporal embedding does not jointly repair the opened event and network metrics. Its lower conditional RMSE coincides with higher cardinality error, so it cannot be interpreted as a localization gain. Close the frozen policy or revise its mechanism without selecting parameters on this tracking outcome.

## Evidence boundary

The V272 event screen reuses the frozen V259 predecision V242 posterior and selected-route history at t=57, then changes only the V272 sensor-level cross-gateway embedding through t=73. The stored V242 continuation is the paired reference. Truth is used only after execution for scoring. A positive screen can authorize one complete M24 candidate run, but cannot establish validation, cross-scale tracking gain or a paper claim.
