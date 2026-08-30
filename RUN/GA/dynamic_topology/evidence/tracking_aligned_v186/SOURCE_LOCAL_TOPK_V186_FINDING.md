# V186 source-local Top-K finding

## Result

On the opened V180 X36 seed-211 t=79 state, limiting every source to its
four lowest-position-uncertainty active labels reduced formation-side synopsis
traffic from 118,992 B to 23,520 B (80.234%).  The uncapped formation selector
chose `[31,24] <- 31` for all six F5 receivers and improved every receiver's
immediate E-OSPA and RMSE.  The Top-4 selector abstained at all six receivers.

## Mechanism

The target label remains within source 31's own Top-4, so the failure is not
caused by dropping the selected source object.  Independent source caps remove
same-label peer summaries at other sources.  That changes the peer support,
shortlist and safety features used by the formation rule, leaving no common
eligible action.  Increasing K would trade the failure against progressively
smaller byte savings without correcting the duplicated communication pattern.

## Next decision

Do not launch the recursive V186 Top-K arm.  Preserve V179 at t=78 and the
formation rule at t=79, but gather source synopses once at a formation
coordinator rather than sending the same source-label inventory independently
to all six receivers.  Each receiver should contribute only the compact local
summary needed for its safety check; the chosen complete label can then be
delivered from the selected physical source to all receivers.  The next
preflight must use a synopsis representation whose charged bytes are sufficient
to reconstruct every runtime feature.

## Evidence boundary

This is an ideal-delivery replay of one opened X36 state.  Truth is used only
after action selection for immediate E-OSPA/RMSE readout.  It rejects the
source-local Top-K compression mechanism and does not constitute validation of
the coordinator alternative.
