# V110 finding: perfect local classification is not enough

## Paired X36 result

| Arm | Mean E-OSPA | Gain vs static | Byte saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V105/V109 all-formation abstention | 79.617863 | +5.259% | +6.117% |
| V110 opened safe-formation oracle | 79.517797 | +5.378% | +3.479% |

V110 improves the aggregate result beyond V105 while keeping positive
consensus gains. F6 returns to the static baseline, including its non-gateway
terminal metric. F1 nevertheless remains slightly negative at `-0.2158%`, so
the registered stability gate still fails.

## Causal finding

V110 never abstains on F1 or F6. Their remaining changes must therefore arrive
indirectly: protected F2--F5 produce different posteriors, and those posteriors
propagate through later full-payload fusion on the static ring. A perfect
classifier of the formation taking the current action is insufficient because
the risk is carried by a time-expanded influence cone, not only by the direct
receiver.

This explains why simply excluding the locally harmed formations improves but
does not certify the network. The next deployable controller needs one of two
mechanisms:

1. predict downstream multi-step risk for every action using the carrier graph;
2. constrain how an altered protected posterior is broadcast outside its
   formation until a recovery condition is met.

The existing V102 evidence is relevant: its alternating one-step-delayed
shield/broadcast schedule produced positive gains for all formations over H=6
(`+4.549%` mean, `+5.021%` byte saving), whereas the later matured-handoff V103
and protection-only V105 schedules lost F1/F6 over H=8. The next minimal
headroom test should therefore extend the V102 alternating broadcast/recovery
cadence to H=8 before introducing a GNN or a label-wise fusion layer.
