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
