# V88 time-expanded path-selective finding

## Paired H=3 result

V88 retained the V86 `acquire -> broadcast -> reference` schedule but changed
only receiver rows that passed the frozen V87 two-hop source gate.  M24
excluded receiver 8; X36 retained all five original broadcast receivers.

| Scale | Broadcast receivers | Mean tracking | Receiver formation | Window consensus | Terminal consensus | Attempted bytes | Strong gate |
|:--|:--|--:|--:|--:|--:|--:|:--:|
| M24 t=104 | `[7, 9, 10, 12]` | +0.059% | F2 +0.209% | -1.795% | -2.066% | -0.063% | fail |
| X36 t=112 | `[19, 20, 21, 22, 24]` | +0.298% | F4 +1.635% | -0.213% | -0.518% | +0.558% | fail |

The X36 mask is identical to V86 and exactly reproduces its tracking result,
confirming the V88 runtime semantics.  Neither scale reaches the frozen 5%
network gate and both worsen consensus.

## V87 proxy falsification

V87 classified M24 receiver 8 as harmful because its protected-label
existence mass decreased.  The paired outcome has the opposite operational
implication.  Under the full V86 broadcast, receivers 7 and 8 improve by
approximately `+0.0129%` and `+0.0125%`; after receiver 8 is excluded, those
gains fall to `+0.0065%` and `+0.0001%`.  M24 position-consensus gain changes
from `-1.557%` to `-5.760%`, while cardinality behavior is unchanged.

The source metric therefore measures posterior change, not task value.  In
particular, it treats every detection-supported label as useful and every
existence decrease as harm, although a supported label can still be a false
track and its suppression can improve E-OSPA.  Large V87 existence nets
cannot be interpreted as predicted tracking gains or used to choose receiver
masks.

## Method decision

Close receiver-selective broadcast based on the V87 sign.  The full-formation
V86 broadcast remains the safer local transport operator: it has positive
affected-formation tracking at both scales and better M24 consensus than the
selective variant.  Its limitation is coverage, not downstream row choice.

The next executable policy changes the temporal and spatial coverage level:
it detects source--gateway handovers from current posteriors, allows
non-conflicting gateways to acquire in parallel, broadcasts each acquired
gateway to its formation on the next round, and repeats this causal cycle
over the full episode with reference fallback.  Message-count, per-row
weight-multiset, physical-edge and rolling-B3 constraints remain unchanged.
The 5% target is evaluated only on that repeated full-episode policy, not on
another isolated H=3 receiver-mask variant.

V88 is opened development evidence.  It does not support validation,
generalization or model-training claims.
