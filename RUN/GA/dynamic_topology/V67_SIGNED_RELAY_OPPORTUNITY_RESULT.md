# V67 signed relay opportunity result

The V67 source-only diagnostic separated the two consequences of withholding
the cross-formation inputs that are already present in the registered reference
route.  All 25 relay states remain below the unchanged 1% material-pressure
budget in both directions.

| Direction | Maximum state | Pressure | Opposite pressure | Signed balance |
|:--|--:|--:|--:|--:|
| quarantine | t=44 | `0.839%` | transport `0.059%` | `-0.779%` |
| transport retention | t=124 | `0.781%` | quarantine `0.053%` | `+0.728%` |

The t=124 direction reversal is informative: late in the relay sequence, the
current cross-formation input is predominantly carrying sender-supported
labels rather than suppressing receiver-supported labels.  It is still below
the frozen material threshold, so V67 authorizes no action and no tracking
run.

## Interpretation boundary

V67 only scores reference edges that already exist.  It therefore cannot
establish that the scene has no positive transport headroom: a more useful
physical sender may be absent from the registered route.  The next diagnostic
must evaluate alternative physical cross-formation sender-to-receiver edges at
t=124, using label-wise projected-Gaussian KLA existence changes and the same
network denominator.  An optimistic fixed-budget upper bound above 1% would
justify building a connectivity-safe transport action; an upper bound below
1% would close this relay seed without opening tracking.

Generated evidence:
`RUN/GA/dynamic_topology/evidence/tracking_aligned_v67/signed_relay_opportunity/SIGNED_CROSS_FORMATION_OPPORTUNITY_V67_RELAY.md`.

Evidence boundary: current-reference edge diagnostic only; no tracking,
alternative-route, validation, or learned-model claim.
