# V73 receiver-fusion alignment design

V72 selected routes with a moment-matched source counterfactual but executed
them with the mixture-aware heavy-fusion receiver. That mismatch must be
removed before attributing the failed tracking result to temporal propagation.

V73 opens only the two existing source anchors: M24 merge-split seed 1401 at
t=80 and X36 merge-split seed 1401 at t=52. It does not execute a route and
does not read tracking outcomes. For every reference and sender-replacement
counterfactual, it uses the same `buildMixtureAwareKlaReferenceConfig` as the
formal filter, orders the receiver's own posterior first, and marks every
selected neighbor as a delivered heavy message. The candidate edge's current
delivery reliability continues to scale its conditional benefit and harm.

The scale-consistent V70 normalization and the V71 physical, message-count,
row-weight, decision-retention, and rolling-B3 projection are then applied
unchanged. The diagnostic question is whether receiver alignment changes the
nominated formations, selected receiver slots, or final route.

If the aligned route differs, the direct utility must be corrected and frozen
before another paired tracking run. If the aligned route and ordering remain
the same, the receiver mismatch is not the dominant explanation and V74 may
add a source-only temporal propagation term. In neither case may V73 support a
tracking, validation, generalization, or learned-model claim.
