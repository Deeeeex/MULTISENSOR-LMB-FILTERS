# Figure 1 Prompt V2

## Goal

Create a paper-ready system overview diagram for the distributed GA-LMB / KLA fusion paper, with clearer narrative emphasis than the current version.

The figure should immediately communicate four ideas:

1. the system is peer-to-peer and distributed, not centralized,
2. the communication graph is heterogeneous,
3. each node first forms a local LMB posterior,
4. the adaptive branch-decoupled KLA fusion module runs locally at every node, with one representative node shown in expanded form.

## Narrative Priority

The figure should read as:

`distributed network setting -> representative node local posterior -> adaptive neighborhood fusion at that node -> spatial/existence branches -> fused local posterior`

Do not make the figure read as a global central fusion controller.

## Recommended Layout

Use a two-part composition.

### Left: Network Setting

Show the dual-formation eight-sensor network:

- two spatial groups of four sensors each,
- sparse inter-group connectivity,
- one representative node `s` highlighted,
- heterogeneous communication links encoded visually.

The left panel should make it obvious that packet delivery quality differs across links. Use three visually distinct link styles, for example:

- solid blue: reliable,
- dashed blue: moderate,
- dotted blue: lossy.

### Right: Representative Local Fusion At Node `s`

Show the processing executed locally at the representative node:

- `Measurements`
- `Local LMB posterior`
- `Neighbor posteriors + realized link outcomes`
- `Adaptive KLA fusion at node s`
- `Spatial branch`
- `Existence branch`
- `Fused local posterior at node s`

Inside the adaptive fusion block, indicate the three driving cues using compact labels, not large formulas:

- `covariance`
- `link quality`
- `existence confidence`

Add a short note somewhere unobtrusive:

`The same adaptive fusion module is executed locally at every node; node s is shown as a representative example.`

## Required Visual Semantics

- The representative node should be visually highlighted but still clearly embedded in the full network.
- The right-side fusion module must be understood as local to node `s`, not global.
- The figure should make the distinction between:
  - local posterior formation,
  - neighbor communication,
  - adaptive fusion-weight allocation.
- The branch split should be visible but lightweight; Figure 1 is a system overview, not the full factorization figure.

## Style Constraints

- academic journal style
- white background
- flat vector design
- thin strokes
- restrained blue-gray palette for local filtering and communication
- restrained dark red accents for adaptive fusion and branch outputs
- consistent typography
- no gradients, shadows, or 3D styling

## Suggested Labels

- `Dual-formation eight-sensor network`
- `Representative node s`
- `Measurements`
- `Local LMB posterior`
- `Neighbor posteriors`
- `Realized link outcomes`
- `Adaptive KLA fusion at node s`
- `Spatial branch`
- `Existence branch`
- `Fused local posterior`

## Avoid

- Do not draw a single large block that appears to fuse all eight nodes centrally.
- Do not repeat the same full three-box pipeline for all eight sensors.
- Do not duplicate Figure 2 by showing the full factorization derivation inside Figure 1.
- Do not over-emphasize topology at the expense of the signal-processing pipeline.
- Do not use decorative icons, sensor cartoons, or slide-style infographic effects.
