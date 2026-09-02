# V244 information-coupled formation braid

## Problem found in the original braid

The original formation-braid scene is a valid structural stress case: the
physical formation graph stays connected while an initially selected sparse
tree becomes infeasible.  However, its sensor formations and target cohorts
use the same independent pair decomposition, `(F1,F2), (F3,F4), ...`.
Consequently a bridge between two modules, such as `F2--F3`, can fail even
though no registered target cohort crosses that cut.  Restoring strong graph
connectivity can then improve consensus without carrying information that
changes E-OSPA or position RMSE.

This is a scene-design confound, not evidence that dynamic routing is useless.
A tracking experiment needs both conditions:

1. the fixed communication backbone loses a physically necessary edge; and
2. target information must cross the affected cut during the same episode.

## Coupled target construction

V244 keeps the sensor trajectories, 120-degree/300 m FoV, 270 m communication
range, target count, and target speed scale of the original formation braid.
It changes the target source/destination mapping.  For `F` formations, target
groups `1..F-1` move from formation `i` to `i+1`, and the final group moves
from `F` to `F-1`.  Thus every adjacent cut of the initial chain has at least
one target handoff while the number of target groups remains exactly `F` at
M24, X36, and X48.

The final return stream uses a separate `+240 m` service lane instead of the
ordinary `+170 m` lane.  Without that single exception, the `F-2 -> F-1` and
`F -> F-1` streams converge in the same lane and violate the existing target
separation gate.  The offset was selected before any tracking outcome was
opened; at all three scales it preserves the registered handover and passes
the unchanged target and sensor--target separation thresholds.

The original scene remains useful as a structural control.  The coupled scene
is the primary task-relevance stress case; convoy and relay remain held-out
style controls after a method is frozen.

## Pre-registered scene gate

Before any tracking result is opened, each M24/X36/X48 coupled scene must:

- pass the existing geometry validation;
- keep the physical formation graph connected for all 160 steps;
- make the initial formation tree infeasible at least once;
- have a target handoff crossing every initial tree cut; and
- in particular, cover every initial tree edge that becomes unavailable.

The V244 preflight uses geometry and registered target routes only.  It does
not authorize tracking claims or tune a method using tracking outcomes.

## Method implication

If V241 succeeds even on the original uncoupled scene, V244 becomes a stronger
cross-check that the gain grows when restored edges carry task-relevant
information.  If V241 improves only consensus, the coupled scene separates a
weak method from a weak task coupling: first rerun the same frozen causal
repair on V244, then add posterior/visibility-aware edge values only if the
coupled comparison still fails.  This avoids changing the method and the
scene at the same time.
