# V117: same-source alternative F6 gateway

## Purpose

V116 shows that even truth-ranked sender-27 labels cannot repair F6 through
the registered `27 -> 32` entry.  The captured pages-5--8 trajectory gives a
more specific diagnosis: sensor 27 is already the lowest-error F5 source,
while sensor 32 is the highest-error F6 receiver.  V117 asks whether the
remaining loss is caused by where the same useful F5 posterior enters F6.

## Frozen candidates

Sender 27, the clockwise formation cycle, the F2--F5 abstention schedule and
the cross-residual weight `0.05` remain fixed.  Receiver 36 has the lowest
captured error, but its baseline residual duplicates a dominant edge; moving
the cross slot there would add a message and change the weight multiset.  It
is therefore excluded by the paired carrier contract.  The edge is moved to
the three lowest-error receivers with a distinct removable residual slot:

| Arm | Cross residual | Captured receiver mean E-OSPA |
|:--|:--|--:|
| gateway-33 | `27 -> 33` | 76.895 |
| gateway-35 | `27 -> 35` | 76.926 |
| gateway-34 | `27 -> 34` | 76.937 |

Moving the gateway does not add a message.  The internal residual edge that
the new gateway displaces is removed, and F6's original `33 -> 32` residual
is restored.  Every candidate therefore preserves row-stochastic fusion,
the nonzero weight multiset, message count, physicality, the F5-to-F6
formation edge and rolling B3 requirements.

## Decision rule

The registered CCW full, better-mean CW full, V113 `27 -> 32` full boundary,
V114 empty boundary and V116 top-5 outcomes remain comparison endpoints.
Only the three new gateway arms are executed.  A candidate must achieve at
least five-percent mean and mature-page gain versus CW full, nonnegative
minimum formation and F6-peer terminal gains, nonnegative worst-sensor,
consensus and communication changes, and rolling B3.

A pass establishes gateway-location headroom and authorizes a causal
receiver/influence-cone value model.  Failure closes same-source within-F6
gateway placement and redirects the next screen to a different formation-
level carrier or multi-source entry.  Receiver choice uses opened current
tracking errors and the oracle arm uses future H=8 outcomes, so V117 is
privileged development evidence only.
