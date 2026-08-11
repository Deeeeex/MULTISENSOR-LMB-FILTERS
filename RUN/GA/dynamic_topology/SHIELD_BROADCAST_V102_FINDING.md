# V102 finding: transport must follow posterior maturation

## Matched result

| Arm | Mean E-OSPA | Gain over static | Attempted-byte saving |
|:--|--:|--:|--:|
| Static full payload | 84.581111 | -- | -- |
| V101 three-step hold | 80.700047 | +4.589% | +5.811% |
| V102 shield/broadcast | 80.733928 | +4.549% | +5.021% |

V102 passes the structural and tail checks but not the registered method gate.
Its per-step gains over static are
`[1.216, 3.603, 5.055, 5.965, 5.583, 5.987]%`; the post-propagation minimum is
5.583%, the worst-sensor gain is 13.668%, the window and terminal consensus
gains are 8.579% and 13.997%, and every rolling B3 window remains connected.
The mean gain is still below 5%, and the weakest formation gain remains only
0.165%.  V102 is also 0.042% worse than V101 in candidate E-OSPA.

## What the broadcast actually changes

Formation gains are `[2.419, 3.296, 5.148, 7.276, 9.432, 0.165]%`.  Relative
to V101, the alternating broadcast helps F1, F2 and F5, hurts F3 and F4, and
leaves F6 effectively unchanged.  In F6, the V102 gains at t=77 are:

| Sensor | 31 | 32 (gateway) | 33 | 34 | 35 | 36 |
|--:|--:|--:|--:|--:|--:|--:|
| Gain vs static | -0.014% | +5.632% | -0.029% | -0.019% | +0.005% | +0.008% |

Thus physical reachability, message-count parity and a direct high-weight edge
are still insufficient.  The useful result exists only at the gateway output;
the peers do not receive that newly produced output on the same synchronous
fusion page.

## Causal interpretation

Each route page reads sender posteriors available at the start of the fusion
round and produces new receiver posteriors at its end.  A protection action can
therefore create a better gateway posterior on page t, but a simultaneous
gateway-to-peer edge still carries the gateway posterior from before that
page.  F6 makes this boundary visible: after protection begins at t=75, the
gateway gain at t=76 is only 0.020%; its large 5.632% gain is produced at the
end of t=77 and can first be transported at t=78.

V102 delayed broadcast by one page from activation, but it did not delay it
until the protected posterior had completed the registered three-page dwell.
The failure is consequently a phase-order error, not evidence that useful
gateway information is intrinsically unshareable.

## Next method decision

Close larger same-page broadcast sets and do not train a selector on V102.
The next headroom test should implement a causal two-stage pipeline:

1. protect a positive-net gateway for three complete fusion pages;
2. only after the third protected output exists, promote that gateway on a
   later broadcast page;
3. alternate broadcast with reference-recovery pages and keep row message
   counts, weights, cached inputs and communication constraints matched;
4. extend the window far enough to observe the transported result rather than
   judging the handoff on its creation page.

This maturity-aware handoff is the minimum mechanism change supported by the
V101/V102 evidence.  M24 replication, new scenes and a learned controller stay
closed until it either clears the X36 headroom gate or falsifies the remaining
transport hypothesis.
