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
