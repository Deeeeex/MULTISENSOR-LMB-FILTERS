# Role-routing scene evaluation decision

## Primary matrix

After a radial M24/X36 method gate passes, the first non-radial evaluation
uses only the frozen v5 **parallel-convoy** and **linear-relay** families.
Their geometry protocol already passed all 80 absolute realization gates and
all 40 paired M24/X36 gates on twenty unopened seeds per preset.  They retain
the same 120-degree, 300 m sensor envelope, aligned headings within a
formation, and scale-matched sensing load.  Geometry qualification does not
authorize a tracking claim.

The two families test distinct information-flow mechanisms:

- **Parallel convoy** keeps formations moving in broadly aligned directions
  while targets traverse and overtake the sensor columns.  It tests whether a
  frozen policy transfers when useful information moves laterally between
  nearby formations instead of radiating from a surrounded centre.
- **Linear relay** arranges formations along a corridor and sends targets
  through sequential FoV handovers.  It tests whether spatially supported
  labels can traverse multiple hops without being lost during local fusion.

The geometry changes the information-flow pattern, not the sensor hardware or
observation advantage.

## Development and stress scenes

- **Merge-split** remains development-only.  Its current realization has high
  multi-formation visibility and very low physical-edge churn, so it is not a
  clean held-out dynamic-routing test.
- **Curved corridor** is executable and visually distinct, but its current
  physical route is nearly static and its target ownership changes too little.
  It is excluded until relative formation motion is redesigned and frozen.
- **Orthogonal crossing** retains stress-only status.  It measures the failure
  boundary under short multi-direction conflicts and cannot be averaged into
  the primary claim.
- **Braided handover** has useful scale-matched sensing geometry, but rigid
  translation leaves one static physical route over the episode.  It cannot
  establish the value of dynamic physical routing.

## Frozen validation order

1. Pass the same preregistered method gate independently on radial M24 and
   X36.
2. Freeze the complete online policy, thresholds and training state.
3. Evaluate unseen radial seeds.
4. Without retuning, evaluate parallel convoy and linear relay on M24 and
   X36; each scale and scene passes independently.
5. Use orthogonal crossing only as a stress result.
6. Add X48 convoy/relay geometry only after steps 1--4 pass.  X48 must repeat
   local handover modules rather than pack more formations into the same area,
   and requires a new frozen geometry version and unopened seed manifest.

Below-gate schedules, scene-specific retuning and exploratory geometries remain
repository-only.  The main progress document may describe this stable scene
design, but it may call a scene tracking-validated only after the corresponding
frozen aggregate gate passes.
