# V119: exact-budget time-expanded dual-path screen

## Why this action is different

V116--V118 close label, sender and receiver selection on the single
`F5 -> F6` boundary.  They do not test whether F6 needs information from a
different formation.  V95 moved several residual tokens for one round using a
generic current-state utility and then returned to a physical-tree baseline;
its X36 effect was only `+0.252%` and its fixed arm was better.  V119 instead
starts from the opened V113 delayed-return mechanism and tests a small,
time-aligned provenance intervention exactly where F6 first turns negative.

## Structural observation

At X36 `t=72`, the clockwise carrier has six cross-formation residual tokens:

| Token | Formation edge | Weight |
|:--|:--|--:|
| `3 -> 8` | F1 -> F2 | 0.05 |
| `9 -> 14` | F2 -> F3 | 0.05 |
| `15 -> 20` | F3 -> F4 | 0.05 |
| `21 -> 26` | F4 -> F5 | 0.05 |
| `27 -> 32` | F5 -> F6 | 0.05 |
| `33 -> 2` | F6 -> F1 | 0.05 |

The physical graph is complete across formations at this opened page, so every
donor sender can reach every F6 receiver.  Permanently moving any one of the
first four tokens to F6 breaks the instantaneous formation ring.  A one-page
move can still satisfy rolling B3 when surrounded by registered carrier pages.
V119 therefore tests a time-expanded pulse, not a static double-entry graph.

## Frozen action family

The V113 clockwise carrier and F2--F5 abstention schedule stay fixed.  The
original `27 -> 32` F5-to-F6 input also stays fixed.  On exactly one pulse page,
one donor token is removed from its registered receiver and reattached to F6:

```text
donor-only:  remove d <- s (0.05);  receiver d self 0.25 -> 0.30
dual-path:   donor-only + add r <- s (0.05); receiver r self 0.25 -> 0.20
```

The dual-path arm keeps every sender message count and the network message
count exactly equal to V113.  The donor-only arm intentionally has one fewer
message and measures how much of the tracking change comes from suppressing
the registered input rather than adding complementary provenance.

Opened V113 outcomes fix the pulse at page 5 (`t=76`), one page before F6 first
turns negative.  Candidate source formations are F1--F4.  For each source,
the F6 receiver is frozen by a current-posterior compatibility score computed
before future outcomes are read; no truth or later measurement may rank the
receiver.  The frozen choices are `3 -> 33`, `9 -> 34`, `15 -> 34` and
`21 -> 34`, with compatibility scores `0.473355`, `0.099777`, `0.073400` and
`0.190296`.  No duration, page or weight sweep is allowed in V119.

## Admission and decision rule

Before tracking, every dual-path candidate must preserve physicality, row
stochasticity, sender-message parity, the 60-message network budget and every
rolling sensor/formation B3 window.  Its paired donor-only action must preserve
valid row weights and the same rolling B3 windows.

An arm passes only if it simultaneously:

- improves mean E-OSPA by at least 5% versus CW full;
- improves mean E-OSPA over both V114 empty-boundary and its own donor-only
  ablation;
- has nonnegative mature-page, weakest-formation, F6-peer, worst-sensor and
  consensus changes versus CW full;
- does not increase attempted bytes.

A pass establishes formation-provenance headroom and authorizes a causal
horizon-aware path-value model, later implemented behind the same safety
projection.  If every arm either loses to donor-only or retains a negative F6
tail, exact-budget local dual provenance is closed; the next method must
reconstruct the formation-level carrier over multiple pages rather than add
another local entry.

V119 uses an opened X36 seed-211 `t=72`, H=8 trajectory and a pulse page chosen
from opened V113 returns.  It is privileged development evidence only, not a
deployable policy or a validation/generalization result.
