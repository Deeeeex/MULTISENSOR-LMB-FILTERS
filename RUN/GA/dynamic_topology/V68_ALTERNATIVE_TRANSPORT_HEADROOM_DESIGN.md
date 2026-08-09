# V68 alternative transport headroom

V67 found a transport-dominant but sub-threshold reference-edge state at relay
t=124.  Because the reference graph may omit the useful sender, V68 evaluates
the physical cross-formation edges that are currently unused.

For each registered residual cross-formation input slot, V68 replaces only the
sender.  The receiver, residual weight `0.05`, number of messages and every
other source remain fixed.  It then calls the installed label-wise
projected-Gaussian KLA to compare the all-delivered reference and replacement
posteriors.  Sender detection support weights positive existence transport;
receiver or incumbent-sender support weights information loss.  Any supported
downward extraction-threshold crossing rejects the raw edge.

The sum of the best positive replacement per residual slot is deliberately an
optimistic upper bound because the edges have not yet undergone the registered
connectivity projection.  If even this bound is below 1%, no transport route is
implemented for the relay seed.  If it exceeds 1%, the exact sender set must be
committed and passed through physical, message-budget, useful-label and
rolling-connectivity constraints before any tracking outcome is opened.
