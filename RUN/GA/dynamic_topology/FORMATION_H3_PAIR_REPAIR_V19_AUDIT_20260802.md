# M24 H=3 pair-repair sequence audit

## Decision

The six conservative trust-0.30 pair repairs do not make the v18 prefix
strict-feasible.  Only all-reference passes all six terminal targets, so the
strict oracle remains `[1,1,1]` with `0%` gain.

## Provenance and controls

- Generation commit: `ec9541cf580c616456c44d1dea5d28d0a4d8b631`
- Cache generation commit: `c9c6d4dcdc7ad1cb04fb88a22823e99c7fc5bc53`
- Preset / state: `m24-formation-fov / seed 211 / t=72`
- Fixed prefix: `[9,13]`
- Third actions: all six unordered formation pairs at trust 0.30
- Strict-feasible arms: `1/8`
- Runtime failures: none

The combined 19-action bank reproduces `[9,13,1]` exactly, including mean
E-OSPA `55.878100`, consensus `43.307878`, and attempted bytes `5,565,248`.
The pair results therefore differ only in their third action.

| Sequence | Pair | Mean | Min. formation | Worst sensor | Consensus | Attempted | Delivered |
|:--|:--|--:|--:|--:|--:|--:|--:|
| `[9,13,14]` | 1+2 | +7.934% | -0.041% | +0.031% | -6.138% | +0.192% | +0.201% |
| `[9,13,15]` | 1+3 | +5.694% | -0.041% | -0.001% | -5.262% | +0.766% | +0.801% |
| `[9,13,16]` | 1+4 | +8.615% | -0.041% | -0.001% | -3.968% | +0.844% | +0.882% |
| `[9,13,17]` | 2+3 | +5.735% | 0.000% | +0.031% | -4.914% | +0.160% | +0.167% |
| `[9,13,18]` | 2+4 | +8.656% | 0.000% | +0.031% | -3.554% | +0.238% | +0.249% |
| `[9,13,19]` | 3+4 | +6.416% | 0.000% | -0.001% | -2.589% | +0.812% | +0.849% |

The `(3,4)` pair repays approximately `77.5%` of the original consensus debt
and retains a gain above the `3%` strong threshold.  Its only material failure
is consensus; the `-0.001%` worst-sensor value is numerically tiny.  The
`(2,4)` pair preserves more tracking but repairs less consensus.  These two
directions expose a useful coordination trade-off, but neither pair closes
the window safely.

## Consequence

This falsifies the existing pair bank as the final repair mechanism.  It does
not support another scalar trust sweep: the more informative next axis is how
many formations are coordinated.  A final bounded subset-order probe will
append the four three-formation combinations and the all-formation action at
the same conservative trust 0.30.  If that still fails, the next method must
optimize a joint mode vector against predicted window-average consensus debt rather
than enumerate uniform-trust subsets.

This remains outcome-inspected, single-state mechanism evidence.  Seeds
223/227, X36, and final seeds are unopened.
