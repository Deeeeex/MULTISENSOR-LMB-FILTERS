# V145 M24 finding: coarse protection-load gating is closed

V145 fails its frozen M24 gate because intervention-window E-OSPA gain is
only `+3.060%`, below the preregistered `+5%` threshold.  The result remains
a repository-only experiment record: X36 is not opened, and no V145 number is
copied into the main progress document.

## Frozen result

- Source commit: `e687740`.
- Preset / seed / anchor / action: `m24-formation-fov` / `1601` / `95` / `25`.
- Observable schedule: all-R anchor, then W on protected intra-formation edges
  while more than half of formations remain protected, followed by latched
  all-R operation.
- Intervention / full-window / mature-window gain:
  `+3.060% / +6.221% / +6.938%`.
- Minimum sensor / formation gain: `+1.564% / +3.335%`.
- Whole-formation W-to-multiplexed-R rejoin match: `100.000%`.
- Attempted-byte delta: `-0.117%`.
- Reference / working wire roles: `2021 / 99`, with one posterior and no
  auxiliary payload on every attempted edge.
- Output-only predictive fallbacks: `33`.
- Gate: fail.

## What the screen resolves

The observable protection-load rule avoids the large weakest-sensor and
weakest-formation regressions of the fixed `R-W-W-W` cadence and retains the
strong positive full and mature windows.  It therefore confirms that
state-dependent W/R assignment is safer than a global periodic schedule.
However, the formation-level protected fraction is too coarse to place the
working role where the immediate intervention needs it: the registered short
window remains two percentage points below the gate.

This closes hand-designed global cadence and formation-load threshold tuning
on the development seed.  The next decision is not another scalar threshold.
It is a preregistered, rank-equivariant bank of feasible edge-level role
vectors evaluated from the same on-policy dual-lineage state with shared RNG
and an all-R recovery horizon.  Learning is authorized only if this finite
counterfactual bank contains independent gate-safe headroom on both M24 and
X36.  If it does not, posterior-role scheduling is closed rather than hidden
behind a GNN.
