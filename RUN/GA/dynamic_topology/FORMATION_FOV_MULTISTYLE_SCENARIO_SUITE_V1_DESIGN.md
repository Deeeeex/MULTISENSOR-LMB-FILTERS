# Formation-FoV Multistyle Scenario Suite v1

## Purpose

The original formation-FoV scale controls are intentionally symmetric:
multiple formations move around a central monitored region while opposed
target corridors cross near the middle.  This is useful for controlled
M24/X36 scaling, but it can overstate generality because formation order,
physical connectivity and target handover all share a radial structure.

The multistyle suite changes the geometry and resulting information-flow
pattern while keeping the registered sensing envelope fixed:

- total FoV angle: `120 deg`;
- hard sensing range: `300 m`;
- detection probability: `0.88`;
- measurement-noise standard deviation: `7`;
- clutter rate/profile: `4`, uniform over the registered global box;
- sensor-quality profile:
  `formation-shared-120deg-r300-q300-v1`;
- six sensors per formation and the same M24/X36 target loads as the radial
  scale controls.

The boresight law is scene dependent but remains shared by every sensor in a
formation.  Moving formations look along their formation velocity; stationary
relay formations use one fixed downward-looking boresight.  Thus no scene
uses independently scattered sensor directions.

## One-switch presets

| Scale | Preset | Geometry | Dominant information-flow event |
|---|---|---|---|
| M24 | `m24-formation-fov-convoy` | formations move east in parallel lanes | targets overtake local formations and change lanes |
| M24 | `m24-formation-fov-crossing` | horizontal and vertical formations cross at staggered times | short-lived multi-direction overlap near intersections |
| M24 | `m24-formation-fov-relay` | stationary formations form a roadside chain | targets are handed from one local sensing cell to the next |
| X36 | `x36-formation-fov-convoy` | larger three-lane convoy | the same local event repeats across six formations |
| X36 | `x36-formation-fov-crossing` | three horizontal plus three vertical formations | more simultaneous but non-identical intersections |
| X36 | `x36-formation-fov-relay` | longer roadside chain | longer-range sequential handover without radial closure |

The existing `m24-formation-fov` and `x36-formation-fov` presets remain the
radial controlled-development scenes.  They are not renamed and their
generated trajectories are not changed.

## Geometry contracts

The first implementation gate checks only outcome-independent properties:

1. platform count, target count and sensor hardware match the scale contract;
2. maximum sensor speed is at most `15 m/s`;
3. maximum sensor acceleration is at most `2 m/s^2`;
4. maximum target speed is at most `14 m/s`;
5. minimum sensor separation is at least `10 m`;
6. each formation uses one shared, finite FoV heading at every time;
7. the attempted reference graph respects edge and node-degree budgets;
8. the fixed comparison backbone is physical throughout the episode.

Crossing formations are temporally staggered.  Horizontal formations clear
the central intersection before the vertical formations enter it.  This keeps
the crossing information-flow pattern while avoiding an unphysical collision
between two sensor rings.

## Removing the radial baseline assumption

The earlier generic graph constructor always connected formation `i` to
`i+1`, including a final edge from the last formation back to the first.  That
is a reasonable radial-ring control, but it is not a geometry-neutral
reference: a linear relay scene can be physically connected even though its
two end formations cannot communicate directly.

Multistyle presets therefore request a **reliable formation-level minimum
spanning tree (MST)**.  Candidate formation pairs are restricted to links for
which at least one sensor-to-sensor edge is physical for the entire episode;
Kruskal selection then minimizes the registered edge score, and the existing
degree/edge budgets remain enforced.  Legacy presets retain the original
formation ring, so this extension does not silently change old evidence.

This static MST is only the conservative comparison backbone.  It does not
use target truth, posterior outcomes or future task loss.  The learned method
must still decide which safe formation-local routing/trust interventions are
worth applying at runtime.

## Calibration and evidence boundary

All six new presets currently carry
`geometry-implemented-difficulty-gates-unfrozen` and set
`formalValidationAuthorized=false`.  Passing structural tests does not make
them paper evidence.

The next calibration stage must freeze, without looking at tracking outcomes:

- blackout and per-target blackout limits;
- single-formation and multi-formation visibility fractions;
- per-sensor target load;
- target handover count and ownership entropy;
- physical-link availability and blockage/focus overlap;
- snapshot times at which the reference posterior is neither trivial nor
  already irrecoverable.

Only after those gates are fixed may the action-bank oracle and deployable
selector be evaluated.  The final claim should require positive and stable
M24/X36 gains on the radial control **and** on multiple unseen geometry
families; no single favorable style is sufficient.

## Figure contract

- **Core conclusion:** fixed sensor hardware produces three visibly distinct
  information-flow geometries, and each geometry has both an M24 and X36
  realization.
- **Archetype:** schematic-led scene composite.
- **Backend:** Python/Matplotlib only.
- **Final size and formats:** `7.2 x 5.15 in`; editable SVG primary, PDF and
  300-dpi PNG secondary.
- **Panel map:** panels a--c show the X36 convoy, crossing and relay scenes;
  panels d--f show the corresponding M24 scale controls.
- **Hero evidence:** the X36 row exposes the three distinct large-network
  layouts and information-flow patterns.
- **Validation evidence:** the M24 row shows that each layout is a scene
  family rather than a one-off node count.
- **Statistics:** none; the figure is a deterministic geometry/protocol
  schematic, not an outcome plot.
- **Source data:** seed-41 trajectories, shared FoV headings and the generated
  static formation backbone exported directly from the tested scenario code.
- **Integrity note:** no target or sensor position is manually moved during
  plotting; FoV wedges use the registered 120-degree/300-m values.
- **Reviewer risk:** too many individual FoV wedges can obscure the layouts.
  Use low-opacity exact wedges, formation-centre paths and one shared legend;
  do not replace actual sensor origins with a decorative formation-level FoV.
