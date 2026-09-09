# Contracting-subspace admission development

This second candidate family addresses the spatial information discarded by
whole-matrix curvature rejection. It is defined while the first scalar
separation development stage is still finishing. All original nine sequences
and the reviewer-identified 409-frame case are development data. No V2X-Real
tracking outcome has been produced. Preserve every first-family result.

For a source rejected by the original full-matrix PSD guard, whiten the
spatial state with its local prior covariance. Diagonalize the whitened local
posterior precision and retain directions whose posterior precision exceeds
the unit prior precision. In these directions, form the ratio of the
normalized posterior marginal to the prior marginal. The discarded
directions contribute the constant one. This retains a proper marginal
likelihood ratio with PSD added precision; its expectation under the local
prior is one. Use the same numerical 1e-10 scale for equality decisions.
Retain the original full ratio exactly for sources that pass the original
guard. If no contracting direction remains, the spatial ratio is one.

The packet still carries the original 15 Gaussian ratio coefficients. A
receiver reconstructs the local prior from those coefficients and the
received local posterior, then constructs the marginal ratio. No additional
packet field, detection change, learned parameter or admission-strength grid
is introduced. Keep the old aggregate integrability fallback.

Compare three complete-recursion candidates:

1. `projected_space`: use the projected spatial ratios while keeping the old
   full-matrix-guarded existence exponents. This isolates spatial projection.
2. `projected_all`: independently retain every otherwise-eligible scalar
   existence increment, and use projected spatial ratios.
3. `projected_consensus`: retain original scalar exponents; additionally
   restore a rejected positive scalar if a contracting direction survives,
   and rejected negative scalars only when both current participating sources
   have negative increments. Keep the same original support strengths.

First reconstruct old fixed inputs to measure discarded directions and the
effect on set extraction. Use this as a mechanism diagnostic, not a full
recursion result. Then compare the three recursive candidates on the same
nine development sequences using mean reliable/intermittent sequence-macro
OSPA. Exact ties retain the earlier method. Keep all per-sequence errors,
false/missed/localization components and recording-group summaries.

Require analytical numerical checks of the marginal-ratio identity,
integrability, invariance to coordinate changes, exact admitted-source
parity, no-opportunity behavior and existence/spatial separation. Rebuild
saved Gaussian updates independently from the local prior/posterior moments
in Python. Only a complete, independently checked recursive result can
select the final method. Freeze selection before running the external cohort
or the remaining complete V2V4Real comparison.
