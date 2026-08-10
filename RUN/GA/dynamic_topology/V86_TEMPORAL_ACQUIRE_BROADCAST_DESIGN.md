# V86 temporal acquire--broadcast design

## Question

V84 found genuine cross-formation information at one receiver, but a single
`0.05` row substitution produced almost no H=3 network gain.  V85 then showed
that the missing breadth cannot be recovered by summing weak same-round row
substitutions: the useful posterior is concentrated at one gateway.

V86 tests the remaining first-principles mechanism: information must first be
acquired by that gateway and only then be propagated through the receiver
formation.  The action therefore changes **when** an existing high-trust input
is used, rather than increasing its trust or adding messages.

## Frozen H=3 schedule

The reference arm uses `[reference, reference, reference]`.  The candidate arm
uses `[acquire, broadcast, reference]`.

1. **Acquire.** Reuse the frozen V84 primary substitution.  One registered
   `0.05` cross-formation sender is replaced by the current-only, mixture-aware
   KLA-qualified sender at the gateway receiver.
2. **Broadcast.** At the next round, every other receiver in the gateway's
   formation promotes the gateway into its existing `0.70` dominant input
   slot.  The displaced dominant sender is retained in the `0.05` residual
   slot; the former residual sender is removed.  If the gateway already owns
   the residual slot, the `0.70` and `0.05` weights are simply exchanged.
3. **Reference.** Return to the registered current-physical reference.

The gateway's own row remains unchanged in the broadcast round.  Thus a
formation of six sensors changes five receiver rows at both M24 and X36.

## Invariants

- Every non-reference row preserves its exact positive-weight multiset.
- Every round preserves each receiver's in-degree and the total directed
  message count.
- No fusion weight, payload, or communication budget is increased.
- Every selected edge must be physically available when executed.
- The complete three-round sequence must pass sensor- and formation-level
  rolling-B3.
- Candidate construction reads only the opened current posterior, current
  physical graph, current reliability and registered topology history.
- Truth and future measurements are used only for paired H=3 scoring after the
  two action sequences are frozen.

The broadcast action is a temporal transport hypothesis, not a current-round
posterior-safety claim: its sender becomes informative only after the acquire
round.  It is therefore not executable as an isolated action and is opened
only through the exact `[acquire, broadcast, reference]` sequence.

## Decision rule

The same primary anchors and the same 5% strong-gain gate are used at both
scales.  V86 advances only if M24 and X36 both achieve at least 5% mean
tracking improvement, do not worsen the worst sensor, minimum formation,
window consensus or terminal consensus, retain message parity, and pass
rolling-B3.  Otherwise this exact full-formation broadcast is closed; the
next design must learn or infer selective downstream recipients rather than
tune the acquisition weight.
