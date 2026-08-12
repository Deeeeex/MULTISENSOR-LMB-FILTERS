# V131 intermittent light network anchor

## Decision

V126 showed that a safe parallel network state can remove the harmful local
tails of V105, but its static full-payload shadow is privileged and free. V128
showed that a zero-message local anchor loses too much useful network evidence.
V131 tests the narrow middle ground: keep an independently propagated network
anchor, refresh it intermittently with moment-compressed LMB messages, and use
it only at the opened V126 rollback cells.

## Frozen mechanism test

- Scenario: X36 formation-FoV, seed 211, continuation `t=72`, horizon `H=8`.
- Working path: unchanged V105 control-only protection schedule.
- Anchor prediction/update: every node, every page.
- Anchor communication: light moment-matched LMB messages at
  `t = [72, 74, 76, 78]` over all currently active directed edges.
- Delivery: reuses the paired working-path physical-link outcome; it introduces
  no extra random draw.
- Fusion: current topology weights, without light-message attenuation.
- Rollback cells: the opened V126 mask (F2/page 5, F1/pages 6--8,
  F6/pages 7--8).
- Accounting: auxiliary attempted/delivered messages, bytes, and runtime are
  included in the candidate totals. Extra in-memory anchor state is disclosed
  but not yet byte-quantified.

The four refresh pages were selected before execution. A full-frequency light
side channel was costed at about 2.10 MB, exceeding V105's 1.75 MB gross byte
headroom. Alternating refreshes cost about 1.05 MB on the frozen V105 payload
trace and leave about 0.70 MB of expected net headroom.

## Development gate

The candidate must retain positive attempted-byte saving, reach at least 5%
mean and post-maturity E-OSPA gains, keep every formation and formation-time
cell nonnegative, keep F6 peer terminal gain nonnegative, preserve both
consensus gains, and pass B3. Failure remains a repository experiment record;
only a threshold-passing result can enter the canonical progress document.
