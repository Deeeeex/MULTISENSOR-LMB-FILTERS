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

## Result and next method decision

The full five-arm screen passes through `scheduled-control-only`: H=3 mean
tracking improves `10.393%`, worst-sensor tracking improves `36.402%`, window
and terminal consensus improve `24.336%` and `8.820%`, and attempted bytes
decrease `0.225%` after synopsis accounting.  Its per-step tracking gains are
`[14.324%, 12.604%, 4.369%]`, confirming that the temporal schedule carries
the mechanism across the full horizon.

Evidence-based label exceptions do not improve this action.  Keeping only
sender-supported labels reduces H=3 gain to `1.694%` and increases attempted
bytes `0.347%`; adding high-existence labels returns almost exactly to the
reference.  The receiver-need-aware rule reproduces the weak V61 result at
`+0.517%`.

The positive conclusion is therefore narrower and more useful: the physical
carrier graph may remain connected, but the data-plane effective KLA graph
must sometimes suppress the complete cross-formation posterior input.  The
next headroom question is sequence selection on X36, not another label
threshold.  A finite multi-step schedule bank will first test whether the
three existing X36 states contain strong strict actions before any GNN is
trained.
