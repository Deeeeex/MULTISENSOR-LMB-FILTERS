# V57 tracking-aligned spatial attribution result

## Decision

Two-formation source/trust composition is feasible and improves several
constraints simultaneously, but it does not create the missing M24 tracking
headroom.  V57 should stop expanding this source/trust family and move to the
multi-formation protection plus state-driven recovery mechanism that produced
the strongest earlier M24 effects.

## Frozen screen

- preset / seed / time: `m24-formation-fov-convoy / 1201 / 60`;
- horizon: three steps;
- action at step 1: every four-mode vector within Hamming distance two of
  all-reference;
- steps 2--3: registered reference;
- candidate count: 67 (reference, 12 local modes, 54 heterogeneous pairs);
- generation commit: `ebe68df`;
- action construction used no truth or future measurement.

## Result

| Quantity | Result |
|:--|--:|
| Realized positive-mean candidates | 54 / 67 |
| Proxy-positive candidates | 3 / 67 |
| Proxy TP / FP / FN | 3 / 0 / 51 |
| Proxy action agreement | 0.227 |
| Best strict mode vector | `[4,3,1,1]` |
| Best strict mean tracking gain | +1.482% |
| Best strict consensus gain | +0.296% |
| Best strict attempted-byte saving | +1.410% |

The best local formation-1 mode gives +1.313%, and the best useful
formation-2 local mode gives +0.169%.  Their joint vector gives +1.482%,
almost exactly the sum of the isolated effects.  No interaction amplification
appears that could plausibly close the remaining 3.518 percentage points to
the 5% gate.  Other formations mainly contribute worst-sensor or consensus
credit rather than network-mean tracking gain.

The current posterior proxy is also unsuitable as a hard proposal gate: it
misses 51 realized positive actions.  This is a proposal-recall failure, even
though the realized source/trust gains remain too small to justify training a
larger selector.

## Next method step

The next bounded experiment evaluates every current-safe subset of formations
whose registered cross input is temporarily protected, followed by reference
recovery.  It is run on the same three M24 anchors.  This tests the spatial
component of the earlier V35 mechanism without assuming that the old fixed
seed-specific schedule transfers.  If joint protection exposes strong but
temporally decaying gains, V57 will combine it with the existing truth-free
retention-debt and staggered-recovery controller.  If it also lacks headroom,
the action must move below formation-level source/trust routing to a richer
receiver- or label-conditioned information-value intervention.

X36 remains closed until the unchanged M24 headroom gate is cleared.
