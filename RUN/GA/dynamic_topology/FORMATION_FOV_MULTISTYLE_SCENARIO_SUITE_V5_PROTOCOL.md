# Formation-FoV Multistyle Scenario Suite v5 Protocol

## Purpose

v5 converts the committed v4 development decision into an executable
held-out geometry protocol.  It does not change convoy or relay trajectories.
The gate values and exact twenty-seed manifest were preregistered at commit
`e7b4f4d` before any held-out result was opened.

The protocol canonical SHA-256 is
`ec631ed036d4e4693353583bdc5fe02d8c5825f7a9d9e6ae187fcb1d235f0e7a`.
It binds the scene digests, development and validation seed lists, absolute
gates, paired M24/X36 gates, all-pass rule and tracking boundary.

## Scene states

| Style | v5 state | Formal geometry validation | Tracking outcomes |
|:--|:--|:--:|:--:|
| Offset-corridor convoy | `held-out-geometry-gate-frozen-v5` | enabled | disabled |
| Linear relay | `held-out-geometry-gate-frozen-v5` | enabled | disabled |
| Orthogonal crossing | `stress-only-v5` | disabled | disabled |

Convoy and relay embed their absolute blackout, overlap, load ceiling,
handover, ownership and blockage requirements in the hashed scene config.
Target-target and sensor-target safety remain hard validator fields.  The
formal evaluator adds the lower load bound, normalized handover rule and
paired scale checks that cannot be expressed as a single-scene config.

## Fixed manifest

The only held-out seed list accepted by the formal evaluator is

```text
[401, 409, 419, 431, 443, 457, 467, 479, 487, 499,
 509, 521, 541, 557, 569, 577, 587, 599, 607, 617]
```

The four presets produce `80` absolute realizations.  Convoy and relay each
produce `20` paired M24/X36 comparisons, for `40` scale checks in total.
Every absolute row and every paired row must pass.  The evaluator rejects a
missing, reordered or substituted preset/seed manifest.

## Execution safeguards

`runFormationFovMultistyleGeometryValidation`:

1. derives the repository from the validator's own file path, verifies every
   critical function resolves to a tracked file in that worktree, and checks
   that worktree has no tracked dirt or untracked MATLAB source;
2. recomputes the supplied protocol payload SHA rather than trusting its
   claimed digest, and verifies the four registered scene SHA values;
3. binds every record cell to its preset, seed, sensor count, formation
   count, sensors per formation and target count;
4. does not accept a seed override;
5. refuses to disable report writing or the clean-worktree requirement;
6. rechecks the scene registry, source resolution and Git state after the run;
7. collects invalid realizations instead of stopping at the first failed
   single-scene gate, then writes every absolute and paired decision;
8. records all absolute and paired gate inputs plus structural failure tokens;
9. writes a fail-closed execution report if generation or validation raises
   unexpectedly, and treats the opened manifest prefix as consumed;
10. exits with a gate failure if any row fails.

Tests include a manifest mutation, an actual sensor-target safety failure and
a case where both absolute scales pass but the X36/M24 load ratio fails.
They also verify that collected structural failures remain reportable,
non-finite metrics fail closed, record identities cannot be swapped or
rescaled, the blackout/single/multi fractions remain a valid partition, and a
forged protocol digest is rejected.  These checks prevent the protocol from
collapsing into a seed-selectable, average-only or `NaN`-permissive audit.

The absolute scene requirements and formal absolute evaluator use the same
frozen gate-definition function; the scene config converts only the normalized
handover threshold into its corresponding integer count.

## Decision boundary

Passing v5 establishes only that the two non-radial scene families are safe,
observable and sufficiently matched across M24/X36 on the unopened geometry
manifest.  It does not establish a learned routing benefit, tracking benefit,
statistical generalization beyond the manifest or an X36 official outcome.

If v5 fails, the observed seed set cannot be reused after changing a gate or
scene.  A repair requires v6 and a new unopened manifest.  If v5 passes, a
later source-frozen protocol may authorize M24 tracking on radial, convoy and
relay; X36 tracking remains contingent on the same method showing headroom
without scene-specific retuning.
