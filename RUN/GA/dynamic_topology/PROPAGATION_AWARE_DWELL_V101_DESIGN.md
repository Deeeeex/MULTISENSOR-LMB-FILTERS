# V101: propagation-aware minimum dwell

## First-principles diagnosis

V100 shows that the X36 gain persists for six steps, but its formation gains
remain highly uneven.  The static carrier has one cross-formation residual
input per formation.  That input reaches a gateway receiver; the protected
gateway posterior then needs two dominant within-formation KLA edges to reach
the most distant members.  A formation action is therefore not memoryless:
turning it off before this path has been served can interrupt the mechanism
that the selector is trying to create.

The observed V100 selections expose one exact diagnostic.  F6 first becomes
positive-net at t=75 but is switched off at t=76; F5 is also switched off at
t=76.  V100's weakest formation gain is only 0.165%, and its post-propagation
gain falls to 4.623% on the same page.  This motivates a topology-calibrated
minimum dwell rather than a longer evaluation window or another label-support
threshold.

## Frozen headroom action

The minimum dwell is three action pages: the gateway action plus the two-edge
within-formation dominant path.  Applying that rule to V100's already opened,
current-observable selection trace changes only t=76:

| t | V100 online set | V101 dwell set |
|--:|:--|:--|
| 72 | 1,2,4,5 | 1,2,4,5 |
| 73 | 1,2,3,4,5 | 1,2,3,4,5 |
| 74 | 1,2,3,4,5 | 1,2,3,4,5 |
| 75 | 1,2,3,4,5,6 | 1,2,3,4,5,6 |
| 76 | 1,2,3,4 | 1,2,3,4,5,6 |
| 77 | 1,2,3,4,5,6 | 1,2,3,4,5,6 |

The carrier graph, fusion weights, cached posterior, measurements, delivery
uniforms, filter RNG and communication model remain identical to the matched
static full-payload arm.  Selected formations use the same control-only
data-plane action as V100; complete Gaussian mixtures are otherwise retained.

This first run is deliberately a frozen headroom probe, not yet an online
controller.  It uses only V100's observable action trace and does not read V101
truth or outcomes.  If it passes, the next implementation will carry the
activation ages causally at runtime and recompute the raw positive-net set from
the candidate posterior.

## Decision gate

V101 advances only if the six-step mean tracking gain over static reaches 5%,
the post-propagation minimum reaches 5%, the weakest formation improves by at
least 1%, all sensor and consensus tails remain nonnegative, rolling B3 passes,
and attempted bytes do not exceed static.  Failure closes dwell as the missing
mechanism and redirects the method to signed receiver--sender--label influence.
