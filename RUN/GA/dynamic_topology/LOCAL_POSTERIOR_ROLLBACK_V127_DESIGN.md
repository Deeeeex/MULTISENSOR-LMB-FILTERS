# V127 local-posterior rollback

V126 proves that restoring a conservative state at 36 harmful X36 node-time
cells can retain more than 5% network and mature-page gain while eliminating
all local regret.  Its exact static shadow state, however, would require an
unaccounted counterfactual communication and filtering path.

V127 keeps the V126 rollback mask but replaces the state source with the
receiver node's current measurement-updated local posterior, which the
standard filter already computes before communication and KLA fusion.  It adds
no message, no payload byte and no parallel shadow filter.  This isolates
whether an immediately available local anchor is strong enough; the opened
mask means it is not yet an online method.

The strict V126 gate is unchanged: at least 5% aggregate and mature-page gain,
no negative formation, formation-time, target-peer, worst-sensor or consensus
gain, nonnegative fully accounted byte saving, and rolling-B3 safety.  Failure
is retained only as a repository experiment record.
