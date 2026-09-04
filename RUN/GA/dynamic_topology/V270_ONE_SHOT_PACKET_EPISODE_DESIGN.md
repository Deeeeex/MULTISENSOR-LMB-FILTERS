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
