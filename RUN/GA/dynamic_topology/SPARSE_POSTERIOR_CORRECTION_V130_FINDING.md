# V130 finding: same-page sparse correction is still too late

## Registered result

| Metric | V130 sparse hybrid correction | Strict gate |
|:--|--:|:--:|
| Mean E-OSPA gain vs static | +5.089% | pass |
| Minimum mature-page gain | +4.849% | fail |
| Minimum formation gain | -0.878% | fail |
| Minimum formation-time gain | -11.556% | fail |
| F6 peer terminal gain | -2.940% | fail |
| Worst-sensor gain | +14.841% | pass |
| Window / terminal consensus gain | +8.825% / +19.545% | pass |
| Fully accounted attempted-byte saving | +6.115% | pass |

V130 keeps V105 control-only protection by default.  It adds light posterior
input only to the opened F1/F6 risk cells and restores a full input at the
isolated F2 page-five cell.  All three payload types are charged and no shadow
state is maintained.

## Mechanism conclusion

Sparse correction preserves the V105 aggregate headroom much better than the
uniform V129 action (+5.089% versus +1.614%), but it does not remove accumulated
local debt.  F1 remains negative on its tail, F6 peers exactly retain the
V105 terminal loss, and the one-page full F2 restoration creates a large
negative transient.  Correcting a formation on the page where regret is
already visible is therefore too late; the incoming message changes only the
gateway's current fusion and needs time to propagate inside the formation.

V130 fails the registered gate and remains a repository experiment record. It
is not promoted to the canonical progress document.

## Method decision

The next useful upper bound is not another weight or timing sweep on the same
working state.  It should test whether a continuously maintained
network-informed *light anchor* has enough state quality for the exact V126
rollback cells.  Such an anchor runs a parallel static route using only
moment-compressed messages.  Its traffic and computation must be counted.  If
that state source cannot clear local safety while retaining net communication
savings, compressed shadow recovery is closed; if it can, the remaining
method problem is a causal risk trigger plus selective anchor maintenance.
