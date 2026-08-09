# V62: layered formation and label routing

## Evidence-driven correction

V61 proves that a one-step label omission is too weak.  Its strongest action
improves the intervention step by `1.589%` but produces only `+0.517%` over
H=3.  The historical `+10.394%` M24 result at the same state requires the
observable temporal formation schedule `[1,2,4] -> [1,2] -> [4]`.

V62 therefore treats formation and label decisions as two layers rather than
competing alternatives.  The formation layer fixes where and when
cross-formation input is restricted.  The label layer decides which complete
Gaussian-mixture Bernoulli posteriors may still traverse each restricted
carrier edge.  The physical reference graph remains installed, so physical
reachability and rolling carrier connectivity are unchanged; each label sees
its own effective subgraph.

## Positive-control action family

The first state remains `m24-formation-fov / seed 211 / t=104`, with the
already-opened formation schedule above.  Four nonreference payload rules are
compared against full reference messages:

| Rule | Information retained on scheduled restricted inputs |
|:--|:--|
| control only | no target label; only the control synopsis |
| sender supported | labels with current sender association mass at least `0.20` |
| sender supported or high existence | sender-supported labels plus labels with existence at least `0.50` |
| receiver need aware | sender-supported labels, plus labels not currently supported at the receiver |

Every retained label carries all Gaussian-mixture components.  Selection is
recomputed from the current posterior at each of the three steps and pays the
same compact synopsis cost used by V61.  The final gate uses recursive H=3
attempted bytes, not an intervention-step estimate.

The method advances only if at least one nonreference rule achieves `5%`
mean E-OSPA improvement with nonnegative worst-sensor, minimum-formation,
window-consensus, terminal-consensus and attempted-byte outcomes.  This is a
development headroom test; X36 and model training remain closed.
