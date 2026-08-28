# V142 spatially supported label augmentation

## First-principles question

V141 corrected six X36 output cardinalities from 18 to 19 without changing
E-OSPA.  The added hybrid state was assigned at the OSPA cutoff, so its
localization penalty exactly cancelled the lower cardinality penalty.  The
remaining causal question is therefore not whether the output needs one more
state, but whether the reference lineage contains a *spatially useful missing
label state* that the hybrid lineage failed to preserve.

## Frozen mechanism intervention

V142 inherits the V139 post-fusion label readout, predictive output fallback,
route, delivery realization, action, seed and gates.  At a protected output
page it performs exactly one new operation before the inherited predictive
fallback:

1. compute the hybrid and exact-relay conditional-MAP cardinalities;
2. act only when the exact relay cardinality is larger;
3. preserve every existing hybrid output label, mean and covariance;
4. extract the exact relay conditional-MAP output at its own cardinality;
5. identify relay output labels absent from the hybrid output;
6. append at most `k_R-k_H` missing labels with their exact-relay mean and
   covariance, in relay conditional-MAP output order;
7. preserve the hybrid output if the reference cannot supply enough missing
   labels;
8. apply the inherited V139 predictive fallback afterward.

No target truth or future measurement is used, and neither posterior lineage
is mutated.  The exact relay state is nevertheless privileged and requires a
second full payload, so V142 is a mechanism teacher rather than a deployable
communication method.

## Registered interpretation

- If X36 does not improve, sparse output-level state rescue is rejected and
  the useful reference information must act earlier in fusion or over time.
- If X36 improves but M24 is harmed, the state rescue requires an observable
  scale/risk gate before it can be used.
- If both scales pass, the appended label, existence probability, mixture
  state and local evidence define a concrete supervision target for an
  edge-label value model.  A deterministic projection must then enforce the
  effective KLA graph and byte budget, and a single-payload rerun is required
  before any communication-method claim.

Failed or below-gate outcomes remain repository-only.  The main Lark document
is updated only after a stable cross-scale method result or a decisive
mechanism conclusion changes the research direction.
