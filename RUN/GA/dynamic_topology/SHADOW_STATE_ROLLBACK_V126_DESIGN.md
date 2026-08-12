# V126 shadow-state rollback

V105 already clears the aggregate and mature-page 5% thresholds, but a small
set of formation-page cells reverses sign after protected states accumulate.
V106 and V107 stop later protection without removing that accumulated state.

V126 keeps the full V105 topology, weights, protection schedule and candidate
communication. A paired static full-payload arm supplies post-fusion shadow
posteriors. Immediately before state extraction, V126 restores the shadow
posterior only for F2 on page 5, F1 on pages 6--8, and F6 on pages 7--8: the
cells that are negative in the opened V105 trace.

This is a causal upper bound. The rollback timing is outcome-selected and the
shadow state comes from a counterfactual arm; its maintenance cost is not yet
claimed. It passes only with at least 5% aggregate and mature-page gain, no
negative formation-time or terminal-formation gain, no F6-peer, worst-sensor
or consensus regression, nonnegative candidate byte saving, and rolling-B3
safety. Failure remains a repository experiment record only.
