# V135 lineage-isolated relay: early-stop finding

V135 is a repository-only mechanism result.  It does not pass the M24 gate,
so X36 was not run and the result must not be copied into the main document.

## Frozen M24 result

- Action: V134 rank-1/rank-3 formation pair, staggered binary reentry.
- Intervention E-OSPA gain: **+2.582%** (required: at least +5%).
- Full-window / mature-window gain: **+0.159% / +0.000%**.
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
protected gateways retain the short-term gain, so the network-average headroom
falls below the gate.

The next design should therefore transmit W on intra-formation edges and R on
cross-formation edges.  This permits useful local propagation inside the
protected formation while preventing altered state from crossing formation
boundaries.  When a formation returns to full cross-input admission, all of
its W states should rejoin R together.  Payload count and type remain matched
to the full-posterior reference; communication compression remains a later
layer, authorized only after the tracking mechanism passes M24 and X36.
