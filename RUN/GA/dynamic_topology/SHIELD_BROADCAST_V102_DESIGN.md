# V102: protected-gateway broadcast

## Why reachability is not enough

V101 raises the X36 mean gain to 4.589% and the post-propagation floor to
5.188%, yet F6 improves only 0.165%.  At t=77 its cross-input gateway, sensor
32, improves 5.632%, while the other five members remain unchanged or regress
slightly.  The static dominant tree is connected, but repeated KLA does not
preserve the gateway's task benefit along that path.

V102 therefore treats useful information transport as an executed fusion-path
problem, not a graph-distance assumption.  It composes two operations:

1. V101's control-only action protects positive-net gateways from a harmful
   cross-formation residual posterior;
2. one step later, each protected gateway directly becomes the 0.70 dominant
   sender of its physically reachable peers.  The displaced dominant sender
   remains at 0.05 and the old residual sender is removed.

Every affected row retains the reference message count, self weight and
positive-weight multiset.  No extra trust or payload budget is introduced.
Peers already dominated by the gateway keep their reference row.

## Frozen H=6 diagnostic

The protection sets equal the already opened V101 sets.  Broadcast sets are
their one-step delay:

| t | Protected formations | Broadcast gateways from |
|--:|:--|:--|
| 72 | 1,2,4,5 | none |
| 73 | 1,2,3,4,5 | 1,2,4,5 |
| 74 | 1,2,3,4,5 | reference recovery |
| 75 | 1,2,3,4,5,6 | 1,2,3,4,5 |
| 76 | 1,2,3,4,5,6 | reference recovery |
| 77 | 1,2,3,4,5,6 | 1,2,3,4,5,6 |

Static and V102 share the t=72 posterior, measurements, delivery uniforms,
filter RNG, communication model and total directed-message budget.  This is a
frozen mechanism headroom test.  A passing result authorizes a causal online
state machine that recomputes positive-net protection and carries protected
gateway identities forward by one step; failure closes broad gateway
broadcast before any model training.

## Gate

V102 must reach 5% six-step mean gain and 5% post-propagation minimum, improve
the weakest formation by at least 1%, keep worst-sensor and both consensus
tails nonnegative, preserve rolling B3, and not exceed static attempted bytes.
Only after this gate passes may the same composition be tested at M24 and
additional radial windows, then convoy, relay, merge-split and curved-corridor.

## Pre-outcome safety correction

The first executable route was rejected before its tracking result could be
reported: three consecutive broad broadcast pages broke sensor-level rolling
B3 at t=75--77 even though every individual row preserved message and weight
parity.  The corrected schedule alternates broadcast and full-reference
recovery pages.  This retains broad formation coverage and the t=77 F6
broadcast while making all six registered sensor- and formation-level B3
windows strongly connected.  No tracking outcome from the rejected route is
used to choose this correction.
