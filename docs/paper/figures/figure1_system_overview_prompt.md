# Figure 1 Prompt

## Goal

Create a paper-ready system overview diagram for a distributed GA-LMB multi-sensor tracking paper.

## Prompt

Draw a clean academic schematic of a peer-to-peer distributed multi-sensor multi-object tracking system. The scene contains two four-sensor formations, for a total of eight sensors, arranged as two spatial groups with sparse inter-group connectivity. Each sensor node should show three internal stages: local measurements, local LMB posterior update, and neighborhood fusion. Communication links between neighbors should be drawn as directed or bidirectional message-passing arrows. The fusion stage at each node should point to a branch-decoupled adaptive fusion block, and that block should visibly feed two outputs: a spatial-weight path and an existence-weight path. Show a compact FID-FIA cue only on the existence-weight path, without explaining the internal FID-FIA calculation. The overall style should be minimal, publication-ready, white background, thin strokes, restrained colors, and no 3D effects. Use clear labels such as “local measurements,” “measurement-updated LMB posterior,” “Branch-Decoupled Adaptive Fusion,” “spatial-weight path,” and “existence-weight path.” Make the local filtering stage visually distinct from the adaptive fusion-weight allocation stage. The final composition should read left-to-right or top-to-bottom as a signal-processing pipeline rather than as a decorative network sketch.

## Required Elements

- eight sensor nodes grouped as two four-sensor formations
- local measurement update at each node
- neighborhood communication links
- local LMB posterior representation
- branch-decoupled adaptive fusion block
- spatial-weight path output
- existence-weight path output
- paper-style labels and simple legend if needed

## Style Constraints

- academic conference or journal style
- white background
- flat vector look
- consistent typography
- restrained blue-gray and dark red accents only where useful
- no gradients, shadows, icons, or infographic decoration

## Do Not

- do not make it look like a slide deck
- do not use cartoon sensor icons
- do not add unnecessary mathematical notation inside the diagram
- do not over-emphasize topology over the fusion pipeline
