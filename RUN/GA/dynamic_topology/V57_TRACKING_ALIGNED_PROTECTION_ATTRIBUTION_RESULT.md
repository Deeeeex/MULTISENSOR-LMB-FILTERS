# V57 tracking-aligned protection attribution result

## Decision

Complete joint protection over all four formations does not expose meaningful
tracking headroom on the current M24 convoy anchors.  V57 should therefore stop
expanding formation-level source/trust/protection combinations.  The next
method revision must first distinguish states with genuine routing leverage
from states where the reference route is already adequate, and then move the
action below formation level when leverage is present.

## Frozen screen

- preset / seed: `m24-formation-fov-convoy / 1201`;
- decision times: `60`, `100`, and `132`;
- horizon: three steps;
- action at step 1: every one of the `2^4 = 16` formation-protection subsets;
- steps 2--3: registered reference;
- only actions passing retention, payload, and rolling-B3 projection execute;
- generation commit: `4dfbfad`;
- action construction used no truth or future measurement.

## Result

| Decision time | Safe actions | Best mean tracking gain | Best consensus gain | Best attempted-byte saving |
|:--|--:|--:|--:|--:|
| 60 | 16 / 16 | +0.046% | +1.313% | +3.271% |
| 100 | 16 / 16 | +0.304% | +0.275% | +1.581% |
| 132 | 16 / 16 | +0.104% | -0.140% | +1.377% |
| Mean over anchors | -- | +0.151% | -- | -- |

The largest mean tracking gain is only +0.304%, and the three-anchor mean is
+0.151%.  These values are far below the unchanged +5% M24 headroom gate.  At
time 100, adding formations 2 and 4 is almost exactly additive; at the other
two times, additional protected formations mainly change communication or
consensus rather than network-mean tracking.  Thus neither full subset
enumeration nor a more elaborate formation-level selector can create the
missing first-order effect on these anchors.

The protection proxy has high recall here (no false negative in the three
screens), but many proxy-positive actions have negligible or negative realized
tracking value.  This is an objective-calibration problem, not a search-space
coverage problem.

## Consequence for V58

The earlier large V35/V36 gains and the current small V57 gains must not be
treated as contradictory evidence.  They were measured at different states:
the earlier states exhibited cross-formation retention debt and delayed
recovery, whereas the current convoy anchors were sampled uniformly and were
not shown to coincide with a handover, visibility asymmetry, route
obsolescence, or posterior-conflict event.

V58 therefore has two ordered tasks:

1. define a truth-free routing-leverage sentinel from current observables and
   compare old high-gain states with current low-gain states;
2. at detected events, assign value to receiver/label-specific cross inputs
   rather than selecting one mode for an entire formation, followed by the
   existing deterministic graph-safety projection and reference fallback.

Low-leverage states should deliberately return the reference action; they are
not required to exhibit a +5% intervention gain.  The +5% gate applies to the
event-conditioned M24 evaluation set, followed by held-out M24 and X36 tests.

This is development attribution only.  It does not support validation or
generalization claims.
