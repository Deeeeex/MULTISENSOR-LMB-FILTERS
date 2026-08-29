# Conditional direction: output-aligned label-effective LMB-KLA

## Trigger and research question

This direction opens only if the frozen V149 X36 reference-cover diagnostic
fails.  It does not reinterpret any W/R result as a positive method result.
V145 already closes hand-designed whole-posterior W/R timing, while V115 and
V116 close current-confidence and truth-distance label rankings on the tested
boundary.  The remaining question is narrower:

> Under a fixed physical carrier and a hard byte budget, can a time-varying
> edge--label participation graph improve the final recursive tracking output
> of a single-state mixture-aware LMB-KLA filter?

The candidate must be compared with the same-scene, same-seed full-posterior
single-state LMB-KLA reference used by this repository.  Its powered-product
fusion retains Gaussian-mixture structure, while its handling of naturally
different label sets still uses the explicitly documented
`fov-aware-censored` approximation.  The reference is therefore not described
as an exact arbitrary-label-set LMB-KLA.  The candidate and control keep that
natural missing-label rule identical.  Only labels deliberately omitted by
the communication action receive explicit zero participation.  The
lineage-relay W/R states, post-fusion label readout and whole-node predictive
fallback are disabled; they are not part of the new action space or baseline.

## Fusion semantics

For an attempted edge `i -> j` and label `ell`, the action has only two data
semantics:

1. **participate**: transmit the sender's complete Bernoulli Gaussian-mixture
   density for `ell`; or
2. **abstain**: declare that this source has zero KLA weight for `ell` on this
   fusion step.

An omitted label is not imputed as low-existence evidence through
`fov-aware-censored`.  The receiver renormalizes the registered KLA weights
over the sources that explicitly participate for that label.

The current restricted label-whitelist hook is not sufficient to implement
this protocol.  It treats every label absent from a restricted payload as an
explicit abstention, so it cannot distinguish the following two cases:

- the sender did not contain the label in its local posterior; and
- the sender contained the label but deliberately omitted it.

Those cases must remain different because the first retains the reference's
natural missing-label rule, while only the second has zero participation.  A
selective message therefore carries an explicit **omitted-local-label set** in
addition to the complete Bernoulli mixtures that participate.  For a label
missing from the payload, the receiver applies zero weight if and only if its
identifier appears in that set; otherwise it applies the unchanged natural
`fov-aware-censored` rule.  Label identifiers and envelope fields are charged
as attempted bytes.

The runtime must also preserve a successfully delivered envelope when every
local label is omitted.  The present message path drops an empty posterior
before fusion, which would silently turn an all-label abstention into a
whole-neighbor absence.  The new message representation therefore separates
delivery from payload cardinality: a received zero-payload envelope remains a
fusion input with its source weight and explicit omitted-label set.  This is a
required semantic change, not an optional implementation optimization.

The distinction is mathematical, not cosmetic.  For Bernoulli inputs, KLA
combines existence log-odds and the spatial-overlap normalizer.  Assigning an
arbitrary low existence to a non-transmitted source changes both terms;
assigning zero participation weight is the only placeholder-independent
neutral omission rule.

## Minimal implementation boundary

The mixture-aware powered-product kernel does not need to be replaced.  The
smallest defensible implementation changes are:

1. replace the whitelist-as-abstention contract with per-source explicit
   omitted-local-label sets;
2. carry a delivery flag and control envelope independently of whether the
   posterior array is empty;
3. let `fuseLmbPosteriorsByLabel` mask a source only when the current label is
   explicitly listed as omitted, leaving all other missing labels to the
   frozen natural-missing rule;
4. introduce one general edge--label message planner, separate from the
   formation-specific V62 planner, that emits complete Bernoulli mixtures,
   omission metadata and exact byte accounting; and
5. replay candidate and full-payload control through the same ordinary
   single-state predict--update--fusion path with lineage and readout features
   disabled.

A focused semantic test must include all three states for the same source:
participating label, explicitly omitted local label, and naturally absent
label.  It must also cover an all-label-omitted delivered envelope.  Without
those cases, an apparent oracle gain is not admissible evidence.

## Headroom before learning

No GNN is trained first.  A privileged finite-horizon oracle must establish
that this action space has useful cross-scale headroom.

The oracle starts from frozen single-state full-payload reference caches.  It changes
only complete-label participation on physically attempted edges, replays the
ordinary mixture-aware fusion, extraction, prediction and update chain, and
scores the resulting final sensor outputs over a fixed horizon.  Candidate
sets may be searched by deterministic coordinate descent or a bounded beam,
but the reported action is evaluated by an independent replay with the same
measurements, delivery uniforms and filter random numbers as its paired
control.

The loss must include network mean E-OSPA, mature-window E-OSPA, the weakest
sensor and formation, and a recovery-tail term.  Intermediate fused KLD,
current existence, association support and receiver need may be observable
features, but none may substitute for the final-horizon outcome in the
teacher target.  Truth and future outcomes are permitted only inside this
development oracle and must be recorded as privileged.

The headroom gate is independent on M24 and X36:

- at least `+5%` intervention-window mean E-OSPA improvement;
- positive full- and mature-window improvement;
- no material negative weakest-sensor or weakest-formation change;
- no attempted-byte increase, with all headers and requests charged;
- no auxiliary posterior channel; and
- the registered time-expanded label-flow constraint remains feasible.

Failure on either scale closes learning on this action family.  A single
opened anchor or one favorable scale cannot authorize training.

## Deployable approximation, only after oracle success

The online model observes only quantities available before transmission:
sender posterior statistics, the receiver's last acknowledged summary,
label information age, FoV/detection opportunity, link state, byte cost and
time-expanded graph position.  It predicts the finite-horizon marginal value
of each feasible edge--label transmission.  A deterministic projector, not
the learned score, enforces:

- the per-edge byte cap;
- positive self weight and valid per-label KLA rows;
- a bounded-window information path for every active label from an
  observation-supported source to the required formations; and
- fallback to the complete static payload schedule whenever the constrained
  selection is infeasible or model confidence is insufficient.

The model order is analytic score, shallow MLP, then a temporal GNN.  A GNN is
justified only if smaller models leave a reproducible cross-scale gap and its
permutation-equivariant graph representation improves transfer to unseen
node counts.

## Theoretical contribution boundary

Per-label event triggering, adaptive track weights, FoV-based active sensor
sets, task-oriented communication and receiver-centric requests are all prior
art.  The potentially defensible contribution is their constrained
intersection for unknown-correlation LMB-KLA: explicit neutral abstention,
complete mixture-aware Bernoulli messages, a time-varying edge--label graph,
and a deterministic byte/connectivity projection driven by final recursive
tracking risk.

The theory should therefore address two claims only:

1. **neutral omission**: zero per-label participation is necessary and
   sufficient for a missing payload to be independent of an arbitrary
   placeholder density; and
2. **information-flow safety**: positive diagonal weights plus bounded-window
   strong connectivity of each active label graph yield contraction of the
   corresponding KLA log-density consensus, with the full static schedule as
   a feasible fallback.  This claim is limited to a common active label,
   mutually compatible positive densities and uniformly lower-bounded
   nonzero row-stochastic weights; births, deaths and label matching remain
   outside the theorem.

Neither claim implies tracking improvement.  That remains an empirical,
paired cross-scale result.

The experimental comparison must also include the closest adaptive
track-weight / undetected-target treatment from prior consensus-LMB work, or
state plainly when that reproduction is unavailable.  Otherwise an observed
gain over the repository's missing-label approximation cannot by itself be
attributed to communication scheduling.

## Evaluation order

After a joint M24/X36 development headroom pass, freeze the action semantics,
features, projector and model weights.  Validate first on unseen radial seeds,
then without retuning on the qualified parallel-convoy and linear-relay
families.  Orthogonal crossing remains a stress boundary.  X48 is opened only
after both non-radial M24/X36 transfers pass.

Below-gate searches, teacher-only gains and failed model variants remain in
repository records and do not enter the main Lark document or manuscript.
