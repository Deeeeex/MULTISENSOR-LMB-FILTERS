# V58 routing-leverage attribution result

## Decision

The M24 convoy scene is not intrinsically devoid of routing leverage.  The
previous uniformly chosen anchors at times 60, 100, and 132 missed the main
counterfactual retention events.  A truth-free scan of the complete opened
convoy reference trajectory selects fresh times 40, 88, and 128 for paired
H=3 headroom evaluation.

## Opened-state mechanism check

The former posterior-contrast/link-stress score is associated with the seven
previously opened M24 tracking gains, but the counterfactual existence
features are stronger and have a direct KLA interpretation.

| Observable | Pearson correlation with best known H=3 gain | Separates opened strong and weak states |
|:--|--:|:--:|
| Former event score | +0.9164 | yes |
| Maximum formation retention debt | +0.9495 | yes |
| Maximum rescued existence fraction | +0.9500 | yes |
| Maximum rescued decision crossings | +0.9264 | yes |

Across the four radial anchors, maximum debt increases from 2.968% to
10.656%, rescued existence increases from 17.930% to 63.938%, and rescued
decision crossings increase from 2 to 10.  Their known H=3 gains increase
from +1.402% to the +7.764%--+10.394% strong range, apart from the expected
non-monotonic ordering between the two strongest states.  At the old convoy
anchors, maximum debt remains below 1.55% and the known gain remains below
1.5%.

## Full convoy scan

The registered 2% debt rule finds short, separated events rather than a
permanently difficult scene.  The largest fresh peaks are:

| Time | Maximum debt | Rescued existence | Rescued decision crossings | Safe formation action selected by the one-round controller |
|--:|--:|--:|--:|:--|
| 40 | 2.937% | 18.562% | 2 | no |
| 88 | 3.615% | 21.717% | 1 | formation 3 |
| 128 | 3.941% | 23.674% | 1 | formation 2 |

The frozen selection rule excludes the previously opened times, ranks by
maximum debt, and requires at least 12 steps between selected events.  It
therefore chooses `[40, 88, 128]`.  Time 80 has three rescued decision
crossings and remains an important diagnostic, but it is only eight steps
from the larger time-88 debt peak and is not substituted after seeing the
features.

## Consequence

V59 should open exactly these three convoy states and enumerate every safe
formation-protection subset for one step followed by the registered reference
route.  This tests whether better event selection restores at least 5% H=3
tracking headroom without changing the action family.  If high-debt states
still lack sufficient headroom, the next action must move below formation
level to receiver--sender--label decisions; another formation-level selector
would not address the missing mechanism.

This is opened development evidence only.  Feature extraction and event
selection use no target truth, future measurement, or future link outcome;
the previously opened H=3 gains are joined only for mechanism attribution.
