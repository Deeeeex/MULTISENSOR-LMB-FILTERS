# V109 design: explicit label abstention

## Question

V105's control-only edge is not semantically neutral.  A delivered empty heavy
payload is retained as a KLA source, and `fov-aware-censored` missing-label
fusion can replace an omitted label with the configured low-existence bound.
The action therefore mixes two different claims: “this source has no payload
for the label” and “this source provides credible evidence that the label is
absent.”  V108 shows that restoring a few positive labels does not repair the
local loss while the remaining bulk negative-evidence path stays active.

V109 asks whether V105's average gain comes from useful isolation of a harmful
cross source or from mass low-existence evidence.  It introduces a third,
explicit state: abstention.  An abstaining source has zero participation weight
for every omitted label.  It contributes neither a spatial density nor a
Bernoulli absence term to that label's KLA.

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
