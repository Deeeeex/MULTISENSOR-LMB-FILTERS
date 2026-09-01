# V231 budget-adaptive source-offer breadth

## Finding that changed the design

The V230 source-relative score was first tested under an intentionally
favorable condition: every source received the current full posterior of all
six beneficiary sensors.  Even then, a fixed two-offer cap recovered neither
opened t=133 teacher label.  The F5 label `[1,4]` ranked fifth by existence
surprise and eighth by spatial utility; the F6 label `[25,20]` ranked
seventeenth and sixth.  Every single-sensor proxy also missed both labels.

This rejects the fixed top-two ranker, but not source-offer itself.  The error
was assigning the offer layer the wrong job.  A compact offer is a proposal,
not a final KLA decision.  It should retain plausible remote surprises with
high recall; the beneficiary coordinator should make the precise choice after
it can jointly see local need, remote summaries, payload cost, and the safe
projection outcome.

## Byte-derived breadth

V231 gives the source proposal layer at most 35% of spendable communication
credit after the permanent 20% saving reserve.  With `S` eligible sources and
`M` offers per source, the exact worst-case control charge is

`S × (16 B solicitation + 16 B response header + M × 24 B record)`.

`M` is the largest integer that fits this control share, capped at 12 before
opening another tracking outcome.  Half of the records retain existence
surprises and half retain spatial/precision candidates.  The coordinator may
still request only one complete Bernoulli-GM label; if no offered payload fits
the remaining credit or passes the KLA projection, it performs no repair.

At the opened X36 t=133 state, nine sources and 10,400 B admission credit give
`M=12`: six existence and six spatial records per source.  The control charge
is 2,880 B.  Adding either 1,456 B teacher payload leaves 6,064 B certified net
saving, so the broader discovery set remains communication-positive.

The same-state proposal screen now recovers both teacher labels because their
ranks are within the frozen six-per-mode breadth.  This result authorizes only
persistence and evaluation of the actual source-local `receivedCache`.  It is
not yet evidence that the cache is populated at the relevant time, that the
beneficiary coordinator will choose the right offer, or that tracking improves.
