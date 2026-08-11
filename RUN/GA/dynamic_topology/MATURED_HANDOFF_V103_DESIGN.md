# V103: maturity-aware causal handoff

## Why V102 did not transport the useful posterior

V102 broadcasts a protected gateway one page after its first activation.  The
X36 result shows that this is not the relevant delay.  A synchronous fusion
page reads the sender posterior available at the start of the round and writes
the protected receiver posterior at the end.  F6 protection starts at t=75,
but its gateway gain is only 0.020% at t=76 and becomes 5.632% at t=77.  The
useful t=77 gateway posterior can first be consumed by peers on a later page.

V103 therefore separates posterior creation and transport.  Handoff eligibility
requires three completed protection pages, matching the registered V101 dwell.
This is a causal protocol rule, not a threshold fitted to V102 tracking scores.

## Frozen H=8 diagnostic

Protection retains the V101 sets and continues all active formations through
the two added pages.  Each formation is handed off once, on the first eligible
broadcast page that does not violate the alternating recovery cadence:

| t | Protected formations | Matured handoff from |
|--:|:--|:--|
| 72 | 1,2,4,5 | none |
| 73 | 1,2,3,4,5 | none |
| 74 | 1,2,3,4,5 | none |
| 75 | 1,2,3,4,5,6 | 1,2,4,5 |
| 76 | 1,2,3,4,5,6 | reference recovery |
| 77 | 1,2,3,4,5,6 | 3 |
| 78 | 1,2,3,4,5,6 | reference recovery |
| 79 | 1,2,3,4,5,6 | 6 |

The H=8 window is necessary because F6 completes its third protected output at
t=77, the following recovery page preserves rolling B3, and its first valid
handoff is t=79.  Ending at t=77 would score posterior creation but never score
the transport operation being tested.

The handoff reuses V102's row-local route transformation: the matured gateway
becomes the dominant sender, the displaced dominant is retained at residual
weight, and the old residual is removed.  Message counts, weight multisets,
physical reachability, cached inputs and communication constraints remain
matched to the static arm.

## Decision gate

V103 advances only if it reaches 5% H=8 mean gain, keeps every gain from the
first handoff onward above 5%, improves the weakest formation by at least 1%,
and gives the five non-gateway F6 members at least 1% terminal mean gain.  The
worst sensor, window and terminal consensus, attempted bytes and every rolling
B3 window must also be nonnegative/pass.

Passing would authorize a causal online state machine with per-formation
protection age.  Failure would close topology-only gateway handoff and redirect
the method to receiver--sender--label value control, because the experiment
would then have supplied a matured, directly reachable gateway posterior
without lifting its peers.

## Result and closure

V103 lowers H=8 mean E-OSPA from 84.037151 to 79.554740, a 5.334% gain over
the matched static arm, while saving 5.981% attempted bytes.  Every time gain
from the first handoff onward exceeds 5%, both consensus tails improve, and
rolling B3 passes.  This is the first result on the hard X36-t72 line to clear
the network-mean 5% threshold.

It is not a method pass.  Formation gains are
`[-0.945, 4.676, 7.731, 9.079, 11.740, -0.022]%`, and the five non-gateway F6
members regress by 2.948% at the terminal handoff.  F1 also changes from
positive early gains to a -15.752% terminal formation gain after its broad
handoff.  The strict receiver/formation gates therefore reject V103.

Maturation timing solves the network-average phase error but not posterior
compatibility.  A gateway posterior can be useful for its own state and still
be harmful to a particular peer because the complete message applies the same
sender substitution to every label.  Topology-only whole-posterior handoff is
closed.  The next method must retain the successful protection/maturity
controller while making transport decisions at receiver--sender--label
granularity.
