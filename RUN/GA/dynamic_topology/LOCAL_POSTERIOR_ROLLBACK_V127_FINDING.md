# V127 finding: same-step local posterior is not an independent safety anchor

## Registered result

| Metric | V127 local-posterior rollback | Strict gate |
|:--|--:|:--:|
| Mean E-OSPA gain vs static | +5.365% | pass |
| Minimum mature-page gain | +5.868% | pass |
| Minimum formation gain | -0.954% | fail |
| Minimum formation-time gain | -15.874% | fail |
| F6 peer terminal gain | -1.471% | fail |
| Worst-sensor gain | +16.701% | pass |
| Window / terminal consensus gain | +9.332% / +15.740% | pass |
| Fully accounted attempted-byte saving | +6.085% | pass |

V127 keeps the exact V126 rollback mask but replaces the privileged static
shadow state with each receiver's current measurement-updated local posterior.
That posterior is already computed by the standard filter, so the intervention
adds no inter-node messages and needs no parallel static fusion arm.  The
rollback counts remain exactly `[0 0 0 0 6 6 12 12]`.

## Causal conclusion

The same-step local posterior is not an independent safety anchor.  It is
predicted from the previous fused working state, so accumulated protection
bias has already entered it before the current local measurement update.  The
intervention therefore cannot undo the F1 tail: F1 formation-time gain falls
from -5.898% at t=77 to -15.874% at t=79.  It also leaves the F6 peers negative
at the terminal page.  V127 preserves the useful network mean headroom, but it
fails the registered local-safety gate and is not promoted to the canonical
progress document.

## Method decision

The next safety anchor must branch before harmful fused state is accumulated.
A zero-message candidate can checkpoint each node's pre-protection posterior,
then propagate that anchor independently using only the node's own prediction
and measurement update while the working path continues V105 fusion.  This
adds local computation and memory but no posterior traffic.  The opened V126
mask should be retained for one more state-source attribution; only after an
independent local anchor clears the strict gate should the rollback timing be
replaced by a causal observable trigger.
