# V148 byte-projected label-role multiplexing

## Why the decision object changes

V142 established a cross-scale causal upper bound: six rare X36 output cells
need one spatially supported label state from the reference lineage. Its
dual-lineage payload costs about 89% more bytes. V143 then showed that a
single-payload W/R carrier can preserve useful long-horizon working-state
effects without extra messages, but whole-posterior role changes create a
short harmful transition. V146 and V147 close two whole-posterior alternatives:
neither formation/edge role banks nor fixed row reassignment can jointly
provide authority, label protection and graph safety.

The obstruction is structural. A 0.70 whole-posterior role change has enough
authority but moves every label together; a 0.05 residual change is safer but
usually weak. V148 therefore changes the atomic decision from an edge role to
an edge--label role while keeping the physical route and every message
opportunity fixed.

## One-payload label-role construction

On the registered V143 reference page, every edge still sends the complete
reference posterior. On a protected intra-formation working page, the sender
already holds a working posterior W and a periodically refreshed reference
posterior R. V148 constructs exactly one outgoing payload:

1. collect the union of current W/R labels;
2. select at most one complete Gaussian-mixture Bernoulli object per label;
3. choose R when it prevents a downward `r=0.5` loss, choose W when it has
   current positive association support, and otherwise choose the object with
   larger existence-weighted spatial credibility;
4. mark reference decision labels, reference-rescue labels and
   detection-supported W labels as mandatory;
5. if the hybrid payload exceeds the bytes of the paired R payload on that
   same edge, remove only the lowest-priority nonmandatory labels; and
6. if the mandatory set still does not fit, send the unchanged R payload.

The reference lineage does not consume a hybrid working payload. It continues
to be refreshed only on the registered R pages, exactly as in V143. The
working lineage consumes the one hybrid payload. Thus V148 neither hides a
second transmission nor mutates the reference anchor with W-only labels.

The rule uses current sender posteriors and current association metadata. It
does not read target truth, future measurements, receiver outcomes or the
paired reference run. Every selected estimation object retains its complete
GM density; no moment-matched light posterior is used as an equivalent
substitute.

## Communication and graph invariants

- The physical and selected graph, fusion weights, delivery uniforms and
  action/rejoin schedule are exactly those of the paired V143 screen.
- Every attempted edge carries exactly one payload role.
- A hybrid W edge is capped by the bytes of its same-edge R payload before
  delivery; the full-run gate additionally requires no total attempted-byte
  increase against the paired static reference.
- Cross-formation edges remain R-only, so the registered effective reference
  graph and finite-time mixing preflight are unchanged.
- No auxiliary payload or uncharged synopsis is created.

## First headroom gate

V148 reuses the frozen V134 M24/X36 action indices and continuation windows.
M24 is run first. X36 opens only if M24 reaches at least 5% intervention gain,
nonnegative full and mature gains, no registered worst-sensor or
weakest-formation regression, exact rejoin, zero auxiliary bytes and no
attempted-byte increase. The identical construction must pass X36 before any
GNN is trained.

The initial rule is an analytic headroom probe, not the final learned method.
If it has joint headroom, a permutation-equivariant model may replace only the
edge--label value ranking using observable existence, association, covariance,
W/R disagreement, link and support-age features. The deterministic payload,
byte and graph projection remains outside the model. If M24 fails, V148 is
recorded repository-only and X36 stays closed.

## Relation to prior label-selective work

V54/V55 optimized KLD fidelity to a full-message reference and reduced bytes
while worsening X36 tracking. V148 does not reuse that objective: it preserves
the tracking-useful W/R state roles identified by V142--V143 and treats KLA
shape/credibility only as a causal selection feature. Per-label weights or
payload sparsity alone are not claimed as novel. The intended contribution is
the combination of tracking-aligned edge--label role value, exact one-payload
projection and a protected effective KLA backbone under a dynamic budget.
