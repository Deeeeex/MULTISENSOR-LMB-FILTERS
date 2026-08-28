# V143 provisional direction: role-multiplexed single-payload routing

## Why the current output-patch branch cannot be the method

The V139 X36 candidate has 2,916 sensor--page outputs.  Only six effective
V141/V142 undercount cells are available to repair.  Even replacing all six
candidate E-OSPA values by their exact paired-reference values would change
the full-window gain only from `+0.453356%` to `+0.466135%`, an increment of
`0.012779` percentage points.  V142 can identify missing information content,
but cannot meet the requested material full-window gain.

The useful high-gain evidence instead comes from the V105/V132 lineage:
protecting the working state yields more than 5% X36 full-window gain, while a
network-informed reference/anchor state removes or reduces the accumulated
local harm.  The remaining first-principles problem is how to maintain both
roles without sending two posteriors on the same edge.

## Two logical state roles

- `W` (working/task state): uses strong intra-formation inputs and controlled
  cross-formation admission to retain the demonstrated task gain.
- `R` (reference/anchor state): preserves a conservative network-informed
  lineage so protected gateway bias cannot become the only state relayed
  downstream.

The current V136 upper bound sends `R` on every selected edge and adds `W` on
intra-formation edges, causing about 89% extra attempted bytes.  The physical
route already separates high-bandwidth local propagation from low-weight
cross-formation transport.  V143 assigns one state role to each wire
opportunity and alternates the local role in time instead of duplicating the
payload.

## Single-payload construction

1. The fixed two-page cycle begins with a **reference phase**.  Every selected
   edge carries `R`; the same received payload may also refresh the local `W`
   lineage, subject to the registered cross-formation admission action.
2. The next page is a **working phase**.  Intra-formation edges carry `W`,
   while cross-formation edges still carry `R`.  Useful `W` information can
   propagate locally, but an altered `W` state never crosses a formation
   boundary.
3. Once no formation remains protected, every edge carries `R` and ordinary
   whole-formation rejoin semantics replace `W` by `R`.
4. A deterministic projector enforces one payload per attempted edge, the
   frozen edge/message budget, a full-reference page every two pages, and the
   registered per-formation `W` connectivity requirement on working pages.
   In the `R` fusion, nominal weight assigned to an edge carrying `W` is moved
   to the receiver's self weight; a physical loss on an edge assigned to `R`
   retains the registered missing-neighbor rule.  This prevents the role split
   from spuriously amplifying the remaining low-weight cross edge.
5. The node keeps both local lineages.  Output selection may use only current
   local evidence and predictive measurement score; it cannot inspect target
   truth, future outcomes, or a paired alternative run.
6. Low-confidence or infeasible role assignments switch the current and all
   subsequent pages to the all-`R` schedule.  This is a prospective safe
   fallback, not a claim that the multiplexed `R` state is bitwise identical
   to an independently executed static reference.

The first mechanism screen uses the fixed reference-first two-page cycle, not
a learned model.  This determines whether the action space has enough
headroom before any GNN is trained.

The frozen V139 routes make the message accounting exact.  Every page has 40
unique off-diagonal messages on M24 and 60 on X36.  The two-phase rule changes
only which already-available posterior occupies each message, so the attempted
message count remains exactly 40/60 and no auxiliary payload is created.

An earlier fixed edge-class split assigned dominant-only edges permanently to
`W`, residual-only edges permanently to `R`, and alternated only shared edges.
It passed a graph-connectivity check but failed the relevant finite-time
mixing check.  Over the complete saved continuation, the static route's
Dobrushin coefficient is `0.3671` on M24 and `0.5151` on X36, whereas the
residual-only `R` lineage remains at `0.9990` and `0.9999`.  It is therefore
rejected before a tracking run.  Under the reference-first two-phase schedule,
the corresponding coefficients are `0.3735` and `0.5178`; the maximum row
total-variation distance from the static mixing product at the end of the
continuation is `0.00927` and `0.00941`.  This is not a tracking guarantee,
but it establishes that the proposed `R` carrier is a credible finite-horizon
anchor rather than merely a topologically connected graph.

## Learning target after a positive mechanism gate

A permutation-equivariant edge--role GNN predicts the marginal multi-step
task value of opening a working phase or assigning an eligible local edge to
`W` rather than `R`.
Inputs are already available online in the repository: per-label existence
and mixture complexity, local evidence/FoV opportunity, association
confidence, NIS and innovation history, W--R disagreement, edge reliability,
delivery history, formation role, protection age, and normalized downstream
reach.  Labels come from paired multi-step rollouts; KLD is a feature, not the
training objective, because the earlier KLD subset teacher degraded X36
tracking.

The GNN only ranks role assignments.  The deterministic projector owns
physicality, one-payload accounting, rolling `R` connectivity, local `W`
connectivity, and fallback.

## Evidence gates

The first fixed-role mechanism must be evaluated on the high-gain M24 and X36
windows, not only the V139 long-tail output screen.  It advances only if both
scales show material task gain, no worst-sensor or worst-formation regression,
and no attempted-byte increase relative to the paired static full-posterior
route.  Runtime and dual-state memory must be reported separately.

After freezing the role projector and learned scorer, evaluation proceeds on
unseen radial seeds followed by convoy and relay, with merge--split and curved
corridor as family holdouts.  Failed fixed schedules, feature variants, and
training sweeps remain repository-only.

## Falsifiers

- If an exact-mixture `R` refreshed on every other page cannot provide a safe
  anchor, role multiplexing is rejected before learning.
- If the reference-first two-phase cycle destroys the `W` benefit, the action
  space lacks enough temporal headroom; the next action must redesign the
  physical route under the same message budget rather than add payloads.
- If a fixed safe role schedule has headroom but the observable scorer cannot
  retain it across M24/X36, the issue is learnability/calibration rather than
  the communication action space.
