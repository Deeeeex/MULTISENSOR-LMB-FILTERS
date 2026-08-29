# V151 design: temporal label-effective KLA participation

## Why V151 is a different question

V150 measures the downstream effect of one source--receiver--label omission at
one opened time.  That is useful for verifying the corrected fusion semantics
and identifying output-sensitive labels, but it is not yet a dynamic
communication method.  A one-step local action occupies only a small fraction
of an H=8 network-wide score, so failure to reach a 5% aggregate gate would not
by itself rule out repeated label-effective control.

V151 asks whether the same legal action, recomputed from the current candidate
state at every step, can accumulate a material and stable benefit without
changing the physical route or increasing full-horizon communication.

## Online action

For every current cross-formation sender--receiver edge and active sender
label, evaluate two exact one-round mixture-aware LMB-KLA counterfactuals:

1. the label participates with the registered reference weight; and
2. that source explicitly omits the label and the remaining source weights are
   renormalized for this label only.

The selector may use only current posteriors, current topology and current
payload sizes.  It ranks candidates by a signed receiver-task proxy rather
than by posterior change magnitude alone:

- MAP-cardinality and existence-decision movement;
- movement toward a receiver-supported local density;
- spatial and uncertainty change for decision-active labels; and
- immediate payload saving after charging the omission envelope.

Only positive signed candidates enter the deterministic projection.  The
projection keeps at least self participation, limits the omitted fraction on
each edge, preserves the carrier graph and falls back to the complete
reference message whenever a constraint is ambiguous.

## Temporal control

The action is rebuilt from the actual candidate posterior at every active
time; no label schedule is copied from a reference rollout.  A small budget
grid controls how many source--label omissions are admitted per step.  A
periodic complete-reference refresh and short hysteresis are evaluated as
explicit variants so that temporary local savings cannot create persistent
information debt.

The first bounded grid is:

| Parameter | Values |
|:--|:--|
| Global omissions per step | 1, 2, 4, 8 |
| Maximum omitted fraction per edge | 0.10, 0.25 |
| Reference refresh period | none, 3 steps |
| Decision horizon | 8 steps |

This grid is development-only and is frozen before X36 is opened.

## Evaluation order

1. Finish V150 M24 singletons and bundles to quantify one-step headroom and
   identify whether effects are additive, inert or harmful.
2. If V150 is below the aggregate gate but has repeatable positive actions,
   run the V151 bounded M24 temporal grid.
3. Freeze the simplest passing M24 rule and transfer it unchanged to X36.
4. Only after both scales pass, compare against full-payload static routing,
   different-FoV adaptive-track fusion and a faithful STS cached-component
   event trigger at matched bytes.
5. Test the frozen rule without retuning on parallel-convoy, linear-relay and
   target-overlap scenes.

## Gate and stop rule

Each scale independently requires at least 5% mean E-OSPA gain, nonnegative
worst-sensor and minimum-formation gains, nonnegative window and terminal
consensus gains, and no increase in full-horizon attempted bytes.  M24 cannot
compensate for X36.

If no bounded V151 rule reaches material M24 headroom, repeated explicit
label omission is stopped before any GNN work.  If V151 passes M24 but fails
X36 without retuning, the action is not scale-general and is also stopped.
Learning is considered only after a simple current-observable selector has
established the action-space headroom; the GNN would approximate marginal
task value, not replace the deterministic safety projection.
