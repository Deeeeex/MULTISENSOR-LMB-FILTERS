# V191: selective formation release closes the M24 extraction failure

## Paired result

V191 keeps the V99 topology, fusion weights, measurements, link outcomes and
filter RNG unchanged.  At M24 seed 211, t=104, it only removes F4 from the
V99 withheld-payload set for the first page, thereby restoring the ordinary
full posterior to F4 for one fusion round.

| Metric | Static full payload | V99 | V191: release F4 at t=104 |
|:--|--:|--:|--:|
| Mean E-OSPA | 71.664511 | 65.182920 | 66.274291 |
| E-OSPA gain | 0 | +9.044% | +7.521% |
| Mean RMSE | 41.837145 | 40.275111 | 37.394018 |
| RMSE gain | 0 | +3.734% | +10.620% |
| Consensus gain | 0 | +21.104% | +17.429% |
| Attempted-byte saving | 0 | +5.080% | +3.562% |
| F4 RMSE gain | 0 | -126.599% | 0 |

Relative to V99, the release costs 71,448 attempted bytes and 1.674% mean
E-OSPA, but improves mean RMSE by 7.154% and completely removes the F4 RMSE
catastrophe.  The remaining minimum formation RMSE gain is -0.145% in F2, so
this teacher does not yet pass the full development gate.

## Method decision

The V99 omission action is useful, but its per-label existence guard does not
protect the downstream LMB set extraction.  Forced positive-label repairs do
not solve this failure, whereas restoring the normal full posterior does.
The next controller should therefore use a hierarchy:

1. use V99-style payload omission when the predicted LMB set decision is
   stable;
2. release a formation back to the ordinary full-posterior path when a
   set-level cardinality or MAP-label-set stability certificate fails;
3. reserve sparse label-wise KLA for localized positive-evidence deficits.

The release decision must be derived from current posterior and link-state
information.  V191 itself uses a teacher-named F4 input, so it establishes
action-space headroom rather than a deployable policy or validation claim.

