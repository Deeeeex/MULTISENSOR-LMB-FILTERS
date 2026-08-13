# V135 lineage-isolated relay: early-stop finding

V135 is a repository-only mechanism result.  It does not pass the M24 gate,
so X36 was not run and the result must not be copied into the main document.

## Frozen M24 result

- First attribution: V134 rank-1/rank-3 formation pair, staggered binary
  reentry, with **+2.582%** intervention gain.
- Decisive action: all ranked formations, staggered binary reentry.
- Intervention E-OSPA gain: **+3.649%** (required: at least +5%).
- Full-window / mature-window gain: **+0.332% / +0.000%**.
- Minimum sensor / formation gain: **+0.000% / +0.000%**.
- Attempted-byte delta versus the paired full-posterior reference:
  **+0.000000%**.
- Every receiver matched the relay state exactly on every page after it was
  no longer protected; the rejoined-state match rate is **100%**.

## Method decision

The causal W/R split fixes the propagation failure diagnosed in V134: local
protection no longer creates long-tail or cross-formation regressions, and
reentry restores the exact reference lineage without extra messages.  However,
using R on every transmitted edge isolates W too strongly.  Only the two
protected gateways retained the first-screen gain; expanding protection to all
formations raised the intervention gain only from +2.582% to +3.649%.  The
network-average headroom therefore remains below the gate.

The next bounded experiment should therefore propagate W on intra-formation
edges and R on cross-formation edges, while retaining an exact R copy for
whole-formation reentry.  This permits useful local propagation inside the
protected formation while preventing altered state from crossing formation
boundaries.  The exact R copy requires a second intra-formation state payload
and is therefore a **charged mechanism upper bound**, not yet a communication
method.  When a formation returns to full cross-input admission, all of its W
states must rejoin R together.  Only a joint M24/X36 gain justifies designing
a budget-feasible multiplexed or compressed representation.

The all-formation page trace supports this decision.  Network gain rises from
`+3.635%` to `+4.710%` and then `+5.207%` over the first three fully protected
pages, but only sensors 2, 8 and 20 retain any full-window gain.  This is direct
evidence that the action has enough instantaneous headroom and that V135's
binding limitation is spatial propagation rather than another duration or
release threshold.
