# V117: same-source alternative F6 gateway

## Purpose

V116 shows that even truth-ranked sender-27 labels cannot repair F6 through
the registered `27 -> 32` entry.  The captured pages-5--8 trajectory gives a
more specific diagnosis: sensor 27 is already the lowest-error F5 source,
while sensor 32 is the highest-error F6 receiver.  V117 asks whether the
remaining loss is caused by where the same useful F5 posterior enters F6.

## Frozen candidates

The clockwise formation cycle, the F2--F5 abstention schedule and each
cross-residual weight `0.05` remain fixed.  Receiver 36 has the lowest
captured error, but its baseline residual duplicates a dominant edge; moving
the cross slot there would add a message and change the weight multiset.  It
is therefore excluded by the paired carrier contract.  The edge is moved to
the three lowest-error receivers with a distinct removable residual slot:

| Arm | F5→F6 entry | F6→F1 return | Captured receiver mean E-OSPA |
|:--|:--|:--|--:|
| gateway-33 | `27 -> 33` | `34 -> 2` | 76.895 |
| gateway-35 | `27 -> 35` | `36 -> 2` | 76.926 |
| gateway-34 | `27 -> 34` | `35 -> 2` | 76.937 |

Moving only the entry cuts the displaced internal source's sole outward
influence path and fails sensor-level rolling B3.  V117 therefore treats the
entry and return as a paired gateway.  The internal residual edge that the
new entry displaces is removed, F6's original `33 -> 32` residual is restored,
and the displaced source takes over the original `33 -> 2` return edge.  Every
candidate preserves row-stochastic fusion, the nonzero weight multiset,
message count, physicality, both formation-cycle edges and rolling B3.

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
