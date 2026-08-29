# V153 safe graph generalization scene matrix

## Purpose

The radial M24/X36 pair is the controlled scale test, not the whole claim.
Its formations move around one central region, so a method can accidentally
benefit from radial order, central overlap or ring-like physical connectivity.
The repository already contains one-switch scene families that remove those
assumptions while preserving six sensors per formation, a 120-degree FoV, a
300 m hard range and the same detection, noise and clutter settings.

V153 therefore uses a staged scene matrix.  No scene is added merely for
visual variety: every family changes one identifiable information-flow
property, and the same frozen method must transfer without retuning.

## Validation tiers

| Tier | M24 / X36 presets | Information-flow question | Role |
|:--|:--|:--|:--|
| 0 | `m24-formation-fov` / `x36-formation-fov` | Does the safe graph action space scale from four to six formations? | Method and scale gate |
| 1A | `*-formation-fov-convoy` | Can useful information move laterally between parallel moving corridors? | Primary geometry transfer |
| 1B | `*-formation-fov-relay` | Can the method preserve and propagate information over a long non-radial chain? | Primary geometry transfer |
| 2A | `*-formation-fov-crossing` | What happens under short-lived multi-direction conflicts? | Failure-boundary stress test |
| 2B | `*-formation-fov-braided-handover` | Can the graph follow repeated changes in the uniquely observing formation? | Handover mechanism stress test |
| 2C | `*-formation-fov-target-overlap` | Are labels preserved when target cohorts overlap and separate while sensors stay fixed? | Label-confusion stress test |
| 2D | `*-formation-fov-merge-split`, `*-formation-fov-curved-corridor` | Does the method survive changing formation geometry and curved motion? | Secondary robustness only |

`*` denotes the matched M24 or X36 prefix.  Existing scenario code and figure
exporters are reused; no target or sensor path is manually edited for V153.

## Frozen order and claim boundary

1. Tier 0 must pass M24 and X36 independently under the same graph generator,
   value rule and thresholds.
2. The frozen method then transfers without retuning to convoy and relay.
   Both scales and both families must pass before a geometry-general claim is
   made.
3. Crossing, braided handover and target overlap report failure boundaries and
   mechanisms.  Their positive results cannot compensate for a Tier-0 or
   Tier-1 failure.
4. Merge-split and curved-corridor remain secondary robustness scenes because
   their geometry changes several factors at once.
5. X48 is enabled only after Tier 0 and Tier 1 pass; it tests scale extrapolation
   and is not used to repair an X36 failure.

For every activated tier, scene geometry, code version, seed manifest and
metric gate are fixed before tracking outcomes are opened.  The aggregate
table reports mean E-OSPA, worst seed, worst sensor, weakest formation,
consensus, exact message count and attempted bytes.  Unsuccessful candidates
remain repository evidence and do not enter the main progress document.

## Figure assets

The tested scenario code already exports editable figures:

- `formation_fov_multistyle_suite_v2.svg` for convoy, crossing and relay;
- `formation_fov_extended_scenes_v1.svg` for merge-split and curved corridor;
- `target_group_overlap_split_m24.svg` for the overlap/separation mechanism.

All wedges in these figures use the registered 120-degree / 300 m sensing
envelope.  They are geometry illustrations, not outcome figures, and should
remain visually and textually separate from tracking-result plots.
