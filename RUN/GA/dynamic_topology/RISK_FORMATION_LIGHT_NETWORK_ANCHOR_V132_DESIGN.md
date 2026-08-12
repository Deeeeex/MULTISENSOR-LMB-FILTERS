# V132 risk-formation light network anchor

V131 proved that a moment-compressed parallel network state can recover F6 and
improve X36 aggregate E-OSPA without exhausting V105's communication headroom,
but alternating whole-network refreshes left F1 and F2 unsafe. V132 changes
only the allocation of the same nominal auxiliary message count.

- Scenario: X36 formation-FoV, seed 211, `t=72`, `H=8`.
- Working path and rollback cells: identical to V131.
- Anchor local prediction/update: all 36 nodes at all eight pages.
- Anchor messages: incoming rows of F1, F2 and F6 at every page.
- Nominal message count on the frozen route: `30 directed receiver-row inputs
  per page × 8 pages = 240`, equal to V131. The implementation nevertheless
  charges the exact active-edge messages and serialized bytes.
- Delivery and fusion: same paired physical delivery outcomes and registered
  topology weights as V131.

The gate is unchanged: at least 5% mean and mature gain, no negative formation
or formation-time cell, nonnegative F6 peer terminal gain, positive consensus
gains, positive attempted-byte saving, and B3. A failure remains repository
only and closes the parallel compressed-anchor branch.
