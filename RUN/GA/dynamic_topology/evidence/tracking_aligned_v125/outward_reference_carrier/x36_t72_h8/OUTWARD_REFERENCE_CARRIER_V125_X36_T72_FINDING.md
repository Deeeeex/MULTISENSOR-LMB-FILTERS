# V125 finding: one-edge state isolation is causal but insufficient

## Decision

V125 fails the frozen X36 gate and is not promoted to the main progress
document. Mean gain over clockwise static is `+4.221%` and the mature-page
minimum is only `+1.830%`, below the required `+5% / +5%`.

## What the experiment establishes

- The paired clockwise capture is exact: its mean E-OSPA is
  `81.803483592419`, identical to the frozen V113 clockwise arm, with eight
  communication-before-fusion snapshots.
- The reference carrier is applied exactly once on sensor `27 -> 32` on each
  of the eight pages. Topology, fusion weights and all V124 receiver-side
  payload decisions remain unchanged.
- F6 is restored exactly to its clockwise-static trajectory: its mean and
  every page gain become `0%`, versus V124's `-1.495%` mean and `-5.865%`
  terminal regression. Thus the V124 F6 loss is caused by the protected F5
  posterior propagating over the F5-to-F6 edge, not by F6's receiver row.
- This exact recovery contributes only `+0.248` percentage points over V124
  (`+3.973% -> +4.221%`). The remaining gate failure is elsewhere: page five
  is the bottleneck (`+1.830%`), with simultaneous F1/F2/F4 regressions of
  `-1.321% / -5.860% / -3.575%`.

## Method consequence

A single F5-to-F6 shadow carrier is too narrow. The useful abstraction is
broader: a node's locally protected estimate and its cross-formation relay
state need not be the same object. The next method decision should therefore
identify and isolate harmful cross-formation state propagation at multiple
boundaries, while preserving beneficial local protection. Because V125 uses
a counterfactual alternative-arm state, it remains a causal upper bound and
not a deployable method.
