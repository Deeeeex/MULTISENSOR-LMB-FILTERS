# V270 one-shot packet episode

V269 improves E-OSPA and RMSE together through `t=59`, while the first
cardinality/E-OSPA regression appears at `t=60`, when a second packet is
fused.  This leaves one causal ambiguity: the loss may be a delayed effect of
the first spatial intervention, or it may be caused by repeated intervention
within one continuous formation-label risk episode.

V270 admits at most one successfully stored packet for each continuous
`(target formation, label)` episode.  A failed first hop does not consume the
episode.  The latch resets only after the policy becomes inactive or the
episode key changes.  Receiver selection remains V269's maximum current
existence margin, and every source, route, prediction, fusion, eta, and byte
accounting choice remains frozen.

The paired `t=57--73` M24 run is a terminating causal ablation, not a duty-cycle
sweep.  Passing the existing joint gate authorizes a full M24 episode.  If the
same set loss remains after only one action, source-preserving spatial packet
intervention is itself unsafe for this event and the action family is closed.

## Paired result and action-family boundary

V270 executes exactly one final packet fusion at `t=58`; the same episode is
suppressed at `t=58,59`, and a later first-hop attempt at `t=68` is dropped.
Relative to V242, network RMSE improves by `8.820%` and F4 event RMSE by
`23.705%`, while the spliced episode still saves `9.964%` bytes against
corrected static routing.  Removing the repeated `t=60` action cuts the V269
network/F4 E-OSPA regressions from `0.219%/0.892%` to `0.045%/0.184%` and the
consistency regression from `0.287%` to `0.127%`.

The remaining loss is nevertheless systematic and formation-local: F4 is the
only formation with E-OSPA regression (`0.173%`), mean absolute cardinality
error rises from `9.9412` to `9.9706`, and positive set-error differences recur
after the single action.  The first packet therefore has a delayed recursive
effect even without repetition.  V268--V270 establish a genuine, charged
localization-versus-set-accuracy tradeoff, not a joint improvement.  The
source-preserving spatial packet action family is closed; choosing a smaller
share from the opened outcome would be an unregistered threshold sweep rather
than a new method result.

The next paper-facing direction should return to topology/action selection and
predict multi-step set and localization consequences before changing the
recursive LMB state.  Packet-derived localization gain may remain a teacher or
auxiliary value signal, but it is not itself a deployable arm under the current
joint objective.
