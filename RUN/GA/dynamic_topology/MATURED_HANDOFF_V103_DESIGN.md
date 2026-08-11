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
