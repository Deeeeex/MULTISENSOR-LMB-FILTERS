# V139 combined output-only safety

V138 repairs the first harmful same-formation fusion page, but per-label local
KLD does not detect every label combination that later harms set extraction.
V137's next-page W/R predictive score detects that severe continuation, but
its state replacement erases useful W propagation.

V139 composes the two signals without changing either hidden lineage:

1. maintain the V136 formation-isolated W state and exact R relay;
2. apply V138's current post-fusion label readout to the protected node's
   current output;
3. when the fixed zero-margin W-minus-R predictive score is negative, replace
   only that node's current extracted output with exact R;
4. continue the next page from the untouched W state, and retain exact
   whole-formation R reentry.

Both decisions use only the current measurement and posteriors already
available at the receiver.  They use neither target truth nor future
measurements.  No threshold or margin sweep is allowed.

V139 is still a charged dual-payload mechanism upper bound.  The M24 screen
requires at least 5% intervention gain, nonnegative full and mature windows,
nonnegative formation gain, exact reentry, and a minimum sensor gain no lower
than `-0.01%`.  The last value is a pre-registered numerical tolerance: it is
two orders of magnitude smaller than V138's remaining material regression and
does not permit a practically meaningful node loss.  M24 failure closes the
branch; a pass authorizes the paired X36 screen under the same tolerance.
