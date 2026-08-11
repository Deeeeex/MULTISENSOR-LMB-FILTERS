# V114: receiver-side influence-boundary shield

## Causal target

V113 shows a constructive interaction between clockwise transport and F2--F5
posterior abstention, but the fixed cycle moves the delayed loss from F1 to
F6.  The saved paired outcome localizes that loss.  On the clockwise residual
carrier, the altered F5 state follows

```text
26 -> 25 -> 30 -> 29 -> 28 -> 27 -> 32 -> 31 -> 36 -> 35 -> 34 -> 33
                              F5 | F6
```

The only F5-to-F6 residual edge is `27 -> 32`.  F6 is neutral through the
first four pages, becomes mixed on page five, and is negative from page six;
by the terminal page nodes 33--36 are each about nine percent worse than the
clockwise full-payload baseline.  The boundary is therefore specific in
receiver, source and arrival time.

## Frozen action family

V114 keeps the complete V113 clockwise F2--F5 schedule and carrier weights.
It adds only F6 to the abstention schedule:

| Arm | F6 shield pages | Purpose |
|:--|:--|:--|
| arrival shield | 6--8 | block the predicted first harmful arrival |
| early shield | 5--8 | add one page of conservative prediction margin |

Formation-level scheduling is receiver-selective here because the clockwise
carrier contains exactly one cross-formation input to F6: sender 27 to
receiver 32.  All intra-formation messages remain active.  The physical graph,
fusion-weight multiset, directed-message count and rolling-B3 route are
unchanged; only posterior participation on this boundary edge changes.

## Comparison and decision

The V113 counter-clockwise full, clockwise full and unshielded clockwise
mechanism outcomes are reused.  Only the two new arms run.  Each candidate is
gated against clockwise full payload and is also compared incrementally with
the unshielded V113 mechanism.

A candidate passes only with at least five-percent mean and mature-page gain,
nonnegative worst-sensor, every-formation and F6 non-gateway terminal gains,
nonnegative consensus and byte changes, and rolling B3.  A pass establishes
that the harmful time-expanded cone can be contained at one receiver boundary
and authorizes a causal arrival predictor.  If both arms fail by losing useful
F6 information, binary boundary abstention closes and V115 moves to
label-complete gradual entry or a physically reachable alternative source.

The boundary and timing come from opened V113 outcomes.  V114 is therefore an
oracle upper-bound screen, not deployable or generalization evidence.

## Completed decision

Both boundary arms preserve rolling B3 and improve the V113 mechanism, but
neither passes the matched clockwise-full gate.  Shielding on pages 6--8
reaches mean E-OSPA `78.358108` (`+4.212%` versus clockwise full and
`+0.155%` versus V113).  Starting one page earlier reaches `78.319230`
(`+4.259%` and `+0.204%`, respectively) and is the oracle arm.

The early shield raises the worst-sensor gain to `+11.136%`, the window and
terminal consensus gains to `+12.671%` and `+19.586%`, and attempted-byte
saving to `+4.244%`.  However, the mature-page minimum is only `+2.960%`, the
minimum formation remains `-0.311%`, and the F6 non-gateway terminal gain is
still `-5.522%`.  Relative to V113, the F6 tail is repaired by only
`1.571%` even though receiver 32 itself improves by `+7.329%` at the terminal
page.

This separates local boundary protection from downstream state recovery.
Blocking the whole posterior at `27 -> 32` protects the entry receiver but
also removes useful label information needed by the other F6 sensors; moving
the start by one page changes the network mean by only `0.049` percentage
points.  Arrival timing is therefore not the limiting decision variable.
Further timing sweeps and whole-posterior binary shielding are closed.  V115
must preserve useful labels while controlling only the harmful label-wise
influence, with a reachable alternative source retained as the fallback
upper bound.
