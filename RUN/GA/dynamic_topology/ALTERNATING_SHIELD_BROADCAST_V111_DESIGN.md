# V111 design: alternating shield/broadcast over H=8

## Motivation

V110 improves aggregate X36 performance while excluding the locally harmed
formations, but F1 remains negative because changed posteriors travel through
the time-expanded static ring. This makes propagation control, rather than
another local risk classifier, the next causal mechanism to test.

V102 already supplies a positive signal: over H=6, one-step-delayed broadcast
alternating with reference recovery gives every formation a positive gain,
with `+4.549%` mean E-OSPA gain and `+5.021%` byte saving. V103 waits for a
three-page maturity point and V105 removes broadcast; both later lose F1/F6
over H=8. V111 therefore extends the original V102 cadence without using V110
outcomes to choose formations or times.

## Frozen intervention

V111 uses the V105 H=8 protection schedule and explicit source abstention.
Protected gateway states are broadcast within their formation one page later
on alternating pages; the intervening pages restore the static reference
route. Broadcast pages are t73, t75, t77 and t79. Static and candidate arms
share measurements, delivery uniforms, filter RNG, communication accounting
and the frozen full-payload reference outcome.

## Decision gate

The probe must achieve at least 5% mean E-OSPA gain, at least 5% gain on every
post-maturity page, nonnegative formation and F6 peer-terminal gains, positive
communication saving, positive consensus gains and the existing B3 reserve.
Passing would justify making the shield/broadcast cadence online and learning
only its risk trigger. Failure would require an explicit time-expanded risk
model or finer label-wise propagation control.

## Result

V111 reaches mean E-OSPA `79.663569`, a `+5.204%` gain over the matched static
full-payload baseline, while saving `5.483%` of attempted bytes. Every page
from t74 onward exceeds 5% gain, and window/terminal consensus improves by
`10.685% / 20.684%`.

The safety gate nevertheless fails. Formation gains are
`[-0.8158, 5.394, 7.449, 7.853, 11.750, -0.1541]%`, and F6 non-gateway terminal
gain is `-2.955%`. The V102 all-positive H=6 result is therefore transient:
fixed periodic propagation control does not prevent long-horizon downstream
debt. The next action selector must model the time-expanded influence graph and
fall back conservatively when downstream regret is uncertain.
