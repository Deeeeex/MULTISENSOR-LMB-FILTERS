# M24 H=3 coordinated-subset repair audit

## Decision

The pre-registered v20 coordination-order probe fails its strong mechanism
gate.  Only all-reference is strict-feasible among seven sequences, so the
strict oracle remains `[1,1,1]` with `0%` gain.  Uniform trust-0.30 subset
enumeration must stop under the frozen rule.

## Provenance and controls

- Generation commit: `402b9a98fbee3cf785a53a54c8df650647fe8383`
- Cache generation commit: `c9c6d4dcdc7ad1cb04fb88a22823e99c7fc5bc53`
- Preset / state: `m24-formation-fov / seed 211 / t=72`
- Fixed prefix: `[9,13]`
- Third actions: all four triples and the all-formation subset at trust 0.30
- Strict-feasible arms: `1/7`
- Runtime failures: none
- Bank construction time: `73.91 s`

The expanded 24-action bank reproduces `[9,13,1]` at the frozen six-target
vector.  Its first 19 actions are regression-tested to preserve the v19 local
and pair action semantics exactly.

| Sequence | Third-step subset | Mean | Min. formation | Worst sensor | Consensus | Attempted | Delivered |
|:--|:--|--:|--:|--:|--:|--:|--:|
| `[9,13,20]` | 1+2+3 | +5.726% | -0.041% | +0.031% | -5.125% | +0.981% | +1.026% |
| `[9,13,21]` | 1+2+4 | +8.647% | -0.041% | +0.031% | -3.778% | +1.059% | +1.108% |
| `[9,13,22]` | 1+3+4 | +6.407% | -0.041% | -0.001% | -2.837% | +1.633% | +1.707% |
| `[9,13,23]` | 2+3+4 | +6.448% | 0.000% | +0.031% | -2.395% | +1.027% | +1.074% |
| `[9,13,24]` | 1+2+3+4 | +6.439% | -0.041% | +0.031% | -2.637% | +1.848% | +1.933% |

## Finding

Increasing coordination order is helpful but not sufficient.  The 2+3+4
triple repays about 79.1% of the original `-11.486%` consensus debt, compared
with 77.5% for the best v19 pair, while retaining strong tracking and positive
communication savings.  Adding formation 1 does not close the remaining
gap: the all-formation action worsens consensus from `-2.395%` to `-2.637%`
and introduces a `-0.041%` formation-tail loss.

This non-monotonicity is the key mechanism evidence.  The useful repair is
formation-dependent; applying the same trust mode to more formations is not
equivalent to better coordination.  The current posterior-only proxy also
marks none of the six realized-positive nonreference sequences as positive,
so it cannot rank the temporal repairs required here.

## Consequence

No further uniform-trust subset, duration, or scalar tolerance sweep is
authorized.  The next candidate mechanism must:

1. represent one mode choice per formation rather than a shared trust;
2. predict terminal H=3 consensus debt from observable current-state and
   action-sequence features;
3. retain mean tracking, minimum-formation, worst-sensor, attempted-byte, and
   delivered-byte targets as explicit safety constraints;
4. use opened seed 211 only for mechanism development until the selector and
   training protocol are frozen.

This remains outcome-inspected, single-state mechanism evidence.  Seeds
223/227, X36, and final seeds are unopened.
