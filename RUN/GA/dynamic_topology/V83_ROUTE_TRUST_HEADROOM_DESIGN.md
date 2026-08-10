# V83 route-and-trust headroom design

## Why the method changes here

V72 changed the identity of a 0.05 cross-formation residual sender for one
round.  Its paired H=3 effect was correspondingly small: `+0.473%` on M24 and
`-0.290%` on X36.  V77--V82 then tried to make that weak pulse recover
monotonically under an internal centered-energy metric.  V82 closes that
route: X36 retains a small global-balance response, while no tested M24 action
contracts.

The first-principles defect is action authority.  A sender selected for useful
complementary information still receives only 0.05 KLA weight, while a fixed
within-formation sender receives 0.70 and receiver self receives 0.25.  Route
selection without trust allocation can identify the right information source
yet be too weak to change tracking materially.  Conversely, replacing the
0.70 backbone would endanger local stability.  The missing control variable is
therefore the trust assigned to a selected safe residual route.

## Frozen headroom action family

V83 keeps the V71 route construction and retains only formations passing the
V75 direct replacement-innovation screen: formation 3 on M24 and formation 4
on X36.  The route, physical edges, two directed messages per receiver, and
0.70 within-formation dominant input are unchanged.  For each affected
receiver, the alternative cross-formation sender receives one trust value

`lambda in {0.05, 0.10, 0.15, 0.20}`.

Increasing `lambda` is funded only from receiver self-weight:

`w_self = 0.25 - (lambda - 0.05)`.

Thus every row remains normalized, no new message is sent, and even the
strongest action preserves positive self-weight and the 0.70 local backbone.
The same four trust values and the same funding rule are used on M24 and X36.

The route cannot be held for all three rounds: the frozen candidate graph
repeated three times fails rolling-B3.  V83 therefore uses the legal sequence
`candidate -> current-physical-tree reference -> candidate`.  The middle
reference round restores the registered information backbone, while two
candidate pulses give a useful sender more than the single weak opportunity
tested by V72.  The complete sequence is frozen from the anchor state; it does
not inspect the intermediate tracking result.  This is an action-space
upper-bound screen, not the final online controller.  If no trust value yields
a material safe gain on one scale, route-and-trust allocation is rejected
before any learned model is built.

## Task-aligned decision gate

For each scale, at least one trust point must satisfy all of the following:

- mean E-OSPA gain at least 5%;
- no worst-sensor or formation-average regression;
- no window or terminal consensus regression;
- attempted bytes within 2% of reference;
- selected rolling-B3 connectivity preserved.

Only if both M24 and X36 pass does the next revision train a variable-size
relative value model.  Its output will be the H-step advantage of each safe
`(route, trust)` action versus reference, not an absolute graph score.  The
deployed selector will abstain to reference unless the predicted lower
confidence bound is positive.  The deterministic physical, message-budget,
backbone, and rolling-connectivity projection remains outside the learned
model.

## Evidence boundary

The two anchors and their future tracking outcomes are already opened
development states.  V83 may establish whether the proposed action family has
enough task-value headroom to justify learning, but cannot support validation,
scene-generalization, or online-policy claims.
