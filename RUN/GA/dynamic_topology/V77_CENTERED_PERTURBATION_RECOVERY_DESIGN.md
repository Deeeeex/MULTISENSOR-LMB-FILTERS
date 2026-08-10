# V77 centered perturbation-recovery design

V76 incorrectly treated every persistent candidate--reference difference as
recovery debt.  V77 uses the standard consensus distinction between the
network common subspace and its centered disagreement subspace.

For each label and each node, V77 forms candidate-minus-reference existence
and position perturbations.  Existence is bounded directly.  Position
perturbations are weighted by shared existence and normalized by a
label-specific pooled covariance.  Across nodes, each perturbation is split
into its network mean and its deviation from that mean.  The salience-weighted
label aggregate gives:

- common energy: information change shared across the network;
- centered energy: node-to-node variation around that shared change.

The decomposition is additive under the shared label metric.  V77 does not
constrain common energy.  Its sole frozen recovery condition is that total
centered energy must be non-increasing from the intervention round through two
fixed-reference KLA rounds.  No absolute tolerance is tuned, and existence and
spatial components are reported separately rather than used as extra gates.

The replay otherwise matches V76: one direct-safe M24 formation-3 or X36
formation-4 pulse, followed by two rounds on the current reference graph, for
both historical and V73 exact slot generations.  It uses current link
reliability but no packet draws, prediction, target motion, measurements,
future links, truth, or tracking outcomes.

If centered energy contracts while the opened V72 outcome still deteriorates,
the missing residual is caused by closed-loop motion/measurement interaction
rather than KLA mixing alone.  That is the decision point for a learned
multi-step residual; the analytic recovery metric must not be retuned to absorb
that exogenous effect.
