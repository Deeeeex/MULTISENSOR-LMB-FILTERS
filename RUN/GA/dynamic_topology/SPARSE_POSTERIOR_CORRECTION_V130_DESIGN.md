# V130 design: sparse hybrid posterior correction

V129 establishes that a light protected input can restore the original F1/F6
formation averages, but uniformly applying it reduces the network gain to
1.614%.  V130 keeps V105 control-only protection everywhere except the opened
V126 risk cells.  It sends a 0.5-weight moment-compressed posterior to F1 on
pages 6--8 and F6 on pages 7--8.  F2's isolated page-five event receives a
full posterior because V129's light input made that transient worse.

This is a privileged action-support attribution: the cell mask comes from
opened V126 outcomes.  It is not an online controller.  It answers a narrower
question before feature or model design: does the available payload action
space contain a sparse correction that retains V105's at-least-5% aggregate
and mature gain while making every formation-time cell nonnegative?  All
control, light, and full payload bytes are charged directly and no shadow
filter is maintained.
