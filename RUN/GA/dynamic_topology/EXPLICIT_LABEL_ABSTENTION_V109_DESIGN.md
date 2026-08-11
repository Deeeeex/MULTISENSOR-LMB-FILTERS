# V109 design: explicit label abstention

## Question

V109 tested the hypothesis that V105's selective empty payload was retained as
a KLA source and interpreted through `fov-aware-censored` missing-label
semantics. If true, V105 would mix source abstention with bulk low-existence
evidence. V109 therefore made abstention explicit: a delivered selective empty
payload would not enter the fusion-input set or refresh the received-label
cache.

## Matched intervention

V109 changes only the post-delivery fusion behavior of V105's selective empty
payload.  The following remain bit-for-bit paired with V105 and the frozen
static baseline:

- static fixed-counter-clockwise adjacency and fusion-weight rows;
- the eight-page V105 formation protection schedule;
- attempted synopsis bytes and full-payload byte reference;
- link attempts, outages, delivery uniforms, measurements and filter RNG; and
- the fov-aware semantics for every non-abstaining message.

When a selective empty payload is delivered, diagnostics and communication
costs still record the delivery, but the empty payload is not inserted into
the receiver's fusion-input set and does not refresh a label cache.  No truth,
future measurement or outcome chooses the action.

## Interpretation

Three outcomes distinguish the mechanism:

1. If V109 preserves the roughly 5% mean gain and removes F1/F6 losses, neutral
   abstention is the correct fallback and the next method should learn when to
   use full positive evidence, credible negative evidence or abstention.
2. If both gain and harm disappear, V105 is mainly a censored-absence effect;
   the method must explicitly select safe negative evidence rather than call
   it communication protection.
3. If the mean gain remains but local harm remains, the loss is caused by
   removing cross-label participation itself and requires label-specific
   effective-graph weights or a different carrier.

The registered gate remains at least 5% mean E-OSPA gain, nonnegative gains for
all six formations and the F6 non-gateway terminal metric, positive byte
saving, positive consensus gains and selected rolling-B3 safety.

## Result and corrected interpretation

V109 exactly reproduces V105 at the reported precision: mean E-OSPA
`79.617863`, gain versus matched static full payload `+5.259%`, communication
saving `+6.117%`, formation gains
`[-0.9312, 4.805, 7.711, 8.970, 11.250, -0.0212]%`, and F6 non-gateway terminal
gain `-2.940%`.

The motivating bulk-negative-evidence hypothesis is false for the exercised
path. In V105, the selective empty payload is already rejected by the
`isempty` gate in `collectCurrentFusionInputs`, before missing-label semantics
are evaluated. V105's benefit therefore comes from source-level abstention,
while its remaining local losses show that formation-wide abstention is too
coarse. The next method should expose label-wise positive, credible-negative
and abstention actions inside the KLA fusion layer.
