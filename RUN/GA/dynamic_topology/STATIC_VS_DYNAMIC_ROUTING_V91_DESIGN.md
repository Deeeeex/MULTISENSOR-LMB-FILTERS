# V91: static versus dynamic routing baseline

## Why this experiment is mandatory

The V89 physical-tree policy is invoked every round and was therefore
initially described as dynamic. That description is insufficient: a policy is
functionally dynamic only if its executed route changes. In the rigidly
translating braided scenes, the physical graph is expected to remain fixed.
V91 first certifies whether the V89 reference is already the matched static
baseline and then reports the adaptive V89 candidate against it.

## Paired arms

- **Static:** freeze the exact route and fusion matrix selected by the dynamic
  physical-tree arm in the first round. They must remain identical over the
  complete episode, and every selected edge must be physical at every time.
- **Dynamic:** the current physical-tree reference, rebuilt from the current
  physical graph at each fusion round.

Both arms use always-heavy posterior exchange, explicit
`support-renormalized` missing-label behavior, two distinct sources per
receiver, weights `{self=0.25, dominant=0.70, residual=0.05}`, `2N` directed
messages per round, and paired measurements, delivery uniforms and filter
randomness.

The preflight first checks whether the dynamic physical-tree arm ever changes
its selected route. If it is identical to the frozen static route for all 160
rounds, then the existing physical-tree outcome is already the static baseline;
rerunning an identical full filter is unnecessary, and any adaptive candidate
can be compared directly against that outcome.

## Reported effect

Dynamic-over-static gain is reported for full-horizon and focus-window
E-OSPA, worst sensor, worst formation, inter-formation disagreement,
cardinality error and bytes. Positive values mean that dynamic routing is
better. A minimum useful signal requires at least `+5%` on both full and focus
E-OSPA with no regression on the worst sensor or worst formation.

This is opened development evidence. It does not authorize a validation or
paper claim, and a failed gate means that subsequent method iterations have
not yet established value over static routing.

## Current braided-scene shortcut

The braided formations translate rigidly with constant relative geometry.
The V89 physical-tree policy reads only the current physical graph and is
deterministic. V91 therefore first certifies the number of unique physical
graphs, selected routes and fusion matrices across all 160 rounds. If all
three counts equal one, the existing V89 reference outcome is already the
matched static baseline and the stored V89 candidate improvement is exactly
the dynamic-over-static result. A duplicate full-filter run is then skipped.

## Baseline hierarchy for richer scenes

Scenes whose all-time physical-graph intersection remains strongly connected
use a frozen, all-time-feasible route as the primary baseline. For a
merge--split or crossing scene where no such route exists, the primary report
must first state that a globally feasible static route is impossible. A frozen
initial route with nonphysical transmissions dropped may then be included as a
stress baseline, but its loss mixes routing quality with loss of feasibility
and cannot be presented as a pure estimation gain. This distinction is
reported separately from tracking metrics.
