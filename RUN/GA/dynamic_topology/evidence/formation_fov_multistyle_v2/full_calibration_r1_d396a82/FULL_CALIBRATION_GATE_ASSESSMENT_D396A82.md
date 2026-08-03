# Multistyle v2 development geometry gate assessment

## Frozen evidence boundary

- Executable scene source: `d396a82`.
- Evidence archive parent: `1121204`.
- Open development seeds: `[41, 43, 47, 53, 59]`.
- Six presets: M24/X36 convoy, crossing and relay.
- All `30/30` realizations pass the structural, kinematic, topology and
  effective-outage validators.
- No tracking filter was run.  This assessment cannot support a routing,
  estimator or cross-scale performance claim.

## Gate decision

| Style | Decision | Main reason |
|:--|:--|:--|
| Parallel convoy | **GO for a separately frozen geometry gate** | M24/X36 have comparable global, focus and target-level blackout, overlap, local load and normalized handovers. |
| Linear relay | **NO-GO** | Low aggregate blackout hides a target-level blind interval of up to `45` consecutive steps and worst-target blackout of up to `36.9%`. |
| Orthogonal crossing | **Stress-only** | Global blackout is `34.7%--39.4%`, worst-target blackout is `71.9%--74.4%`, and multi-formation visibility is only `9.3%--16.3%`. |

The complete `radial + convoy + relay` matrix therefore remains closed.
Convoy success cannot compensate for relay failure, and crossing is not
averaged into the main result.

## Why relay fails despite a low global blackout

Relay global blackout is only `2.2%--3.4%`, and its M24/X36 aggregate
single-/multi-formation fractions and local sensor load are well matched.
Those averages distribute a localized failure across all target-time
samples.  The target-level diagnostics reveal the actual risk:

| Relay metric | M24 range | X36 range | Main-scene requirement |
|:--|--:|--:|--:|
| Worst-target blackout | 28.7%--35.6% | 30.6%--36.9% | at most 23% |
| Longest blackout | 27--34 steps | 18--45 steps | at most 24 steps |
| Focus blackout | 1.0%--1.4% | 1.8%--2.6% | at most 4% |
| Multi-formation visibility | 50.6%--52.3% | 51.5%--54.3% | at least 45% |

A `45`-step blind interval spans about `28%` of the 160-step episode and can
create deletion/rebirth and OSPA spikes independently of the routing policy.
Calling the scene matched merely because both scales contain such a defect
would be a false generalization result.

## Candidate preregistration thresholds

These thresholds are proposed from the opened development evidence.  They
must be committed before any fresh seed is evaluated and must pass for every
seed, not only in aggregate.

| Per-seed metric | Convoy | Relay |
|:--|--:|--:|
| Global blackout | `<= 0.025` | `<= 0.050` |
| Focus blackout | `<= 0.035` | `<= 0.040` |
| Worst-target blackout | `<= 0.080` | `<= 0.230` |
| Longest blackout | `<= 12` steps | `<= 24` steps |
| Single-formation visibility | `>= 0.25` | `>= 0.35` |
| Multi-formation visibility | `>= 0.55` | `>= 0.45` |
| Focus target load per sensor-time | `4.0--5.5` | `4.0--5.5` |
| Ownership entropy | `>= 0.95` | `>= 0.95` |
| Blockage/focus overlap | `>= 0.60` | `>= 0.60` |

Cross-scale comparison is conditional on both absolute gates passing.  For
the same seed, proposed maximum M24/X36 differences are `0.010` global
blackout, `0.015` focus blackout, `0.030` / `0.050` worst-target blackout
for convoy / relay, `4` / `8` longest-blackout steps, `0.10` / `0.08`
single- or multi-formation visibility, and `0.03` ownership entropy.  The
X36/M24 focus-load ratio must lie in `0.90--1.15` for convoy and
`0.85--1.15` for relay.

## Next decision

The relay route/FoV interaction must be diagnosed and repaired under a new
scene contract.  The five opened seeds can be used only for repair and
calibration.  After thresholds and the revised source digest are committed,
geometry validation should use at least twenty unopened seeds with an
all-seeds-pass rule.  A held-out failure creates a new version and a new
unopened set; it does not authorize relaxing the gate after seeing the data.

## Artifacts

The Markdown report is committed.  The MAT file and raw log remain local and
are intentionally excluded by the experiment directory ignore policy.

| Artifact | SHA-256 |
|:--|:--|
| `DYNAMIC_TOPOLOGY_SCENE_DIFFICULTY_20260803_121213.md` | `95d1fb6c5b829dfcefba15fbdd7356a43c9a0a2c053424756fec15bc890f76e1` |
| `DYNAMIC_TOPOLOGY_SCENE_DIFFICULTY_20260803_121213.mat` | `86c85b9d79581a10edba9387862df250c542afa8cbcd9d5a151f3bcd2dec6814` |
| `logs/MULTISTYLE_FULL_V2_CALIBRATION_D396A82.log` | `7c2056536760af644a64af99de2459b1526f29913b5140afdebd3a6a2a1dea7e` |
