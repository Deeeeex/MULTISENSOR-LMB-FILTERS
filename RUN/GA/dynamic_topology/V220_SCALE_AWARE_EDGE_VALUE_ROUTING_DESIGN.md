# V220 scale-aware semantic effective-graph routing

## Decision

V220 does **not** learn a denser full-posterior topology.  It keeps the
registered static full-posterior cycle as the protected fallback and learns a
two-stage, byte-conserving reallocation:

1. select one receiver formation whose ordinary full-posterior consumption can
   be suspended for one page without material tracking harm (the communication
   **donor**);
2. spend only the resulting exact communication credit on one complete-label
   Bernoulli Gaussian-mixture KLA shortcut from an information-rich source to
   an independently selected information-deficient formation (the
   **beneficiary**); and
3. fall back to the unmodified static cycle whenever either stage lacks a
   positive calibrated lower value or fails a deterministic safety check.

The donor and beneficiary are deliberately decoupled.  The earlier V216
controller forced them to be the same formation, although the strongest V206
teacher sequence earned credit by releasing/suppressing one formation and
spent it on labels needed by other formations.  V220 turns that mechanism into
an explicit graph decision rather than preserving the same-formation
restriction.

The learned component ranks a finite bank of executable actions.  It cannot
create a physical link, overspend communication credit, send a moment-matched
surrogate, relax delivery checks, or certify a harmful donor by appealing to a
later repair.

## Why generic full-posterior rerouting is not the method

The earlier V27 audit already tested the tempting alternative: use the same or
similar message budget to increase full-posterior mixing through stronger
cross weights, dual gateways, bidirectional cycles, or antipodal links.  It
found `0/10` strong actions on the opened M24 state.

| V27 action | Mean E-OSPA gain | Consensus gain | Mean cardinality error |
|:--|--:|--:|--:|
| Registered cycle, cross weight 0.05 | +0.000% | +0.000% | 2.708 |
| Dual gateway, cross weight 0.05 | -6.848% | -20.863% | 3.167 |
| Bidirectional cycle, cross weight 0.05 | -5.972% | -16.209% | 3.056 |
| Cycle plus antipodal links, cross weight 0.05 | -8.358% | -24.621% | 3.250 |

All ten nonreference actions reduced the moment-summary posterior
disagreement while increasing cardinality error.  More agreement was therefore
not more correct.  For a Bernoulli label, KLA satisfies

`logit(r_bar_l) = sum_i omega_i logit(r_i,l) + log(eta_l)`, with `eta_l <= 1`.

Spatial conflict makes `log(eta_l)` negative.  Indiscriminate full-posterior
mixing can consequently suppress label existence even while it contracts
inter-sensor disagreement.  Generic cycle rerouting remains a negative/static
ablation; it is not the primary V220 action family.

## Key X36 diagnosis

The X36 seed-1301 reference trajectory separates sensing geometry from
information-flow failure.

| Diagnostic | t=118 | t=132 |
|:--|--:|--:|
| Active / completely invisible targets | 24 / 0 | 24 / 0 |
| Mean visible formations per target | 2.375 | 2.000 |
| Mean visible sensors per target | 11.125 | 10.625 |
| Median per-sensor MAP cardinality | 11 | 10 |
| Median per-sensor cardinality error | 13 | 14 |
| Median posterior object count | 24 | 24 |
| Median expected cardinality, sum of existence probabilities | 11.54 | 10.47 |
| Predecision mean E-OSPA | 113.589 | 118.563 |
| Predecision mean position RMSE | 347.688 | 354.406 |

The posterior container has the required 24-label capacity.  At t=118 every
true label has existence at least 0.5 at one or more sensors, and at t=132 this
holds for 21 of 24 labels.  Label information exists but is strongly
localized: some formations assign high existence to a target group while
others assign almost zero.

At both selected times every formation pair has at least one physical sensor
link.  The reference still schedules only six directed cross-formation
messages per page, following

`F1 -> F2 -> F3 -> F4 -> F5 -> F6 -> F1`.

The cycle has a formation-level propagation diameter of five pages.  The
t=130--131 history also contains missed scheduled cross-formation deliveries.
This makes semantic information propagation, rather than FoV width, posterior
capacity, or formation-level physical disconnection, the leading X36
hypothesis.  Paired counterfactuals are still required to establish causal
tracking gain.

## Executable action bank

An action is

`a = (d, b, s, l)`,

where `d` is the communication-donor formation, `b` is the beneficiary
formation, `s` is a source sensor physically able to serve every receiver in
`b`, and `l` is one complete Bernoulli GM label.  `d` and `b` may differ.

| Arm | Full-posterior backbone | Label shortcut | Role |
|:--|:--|:--|:--|
| Static cycle | unchanged | none | no-op baseline and deployment fallback |
| Donor only | one receiver formation suspends ordinary full-posterior consumption for one page | none | stage-1 causal safety label |
| Donor + semantic shortcut | same one-page suspension, then ordinary fusion | complete label `(s,l)` sent to all receivers in independently chosen `b` | primary V220 action |
| Same-formation donor + repair | donor forced equal to beneficiary | complete label | V216 structural ablation |
| Generic full-posterior mixing | V27-style cycle/multi-gateway alternatives | none | negative topology ablation |
| V206 teacher | opened identifier-driven sequence | complete labels | mechanism upper bound only |

For each beneficiary, the candidate bank is reconstructed from current local
and ordinary-fused posteriors.  It retains only complete-label candidates that
pass physical/common-source reachability, current delivery availability,
nonzero receiver support, position/mode compatibility, rolling recovery, and
exact payload accounting.  A deterministic mode-diverse cap prevents the
bank size from growing with raw sensor count.

The exact ledger enforces

`control_bytes + shortcut_bytes <= spendable_credit(d)`,

after a fixed reserve fraction of the donor saving has been locked.  Protected
savings cannot be spent.  The communication claim is therefore checked in
attempted bytes, independently of whether the shortcut is delivered.

## Causal two-stage target

Every action is replayed from the same cached predecision state for three
pages.  The two value heads have different counterfactual baselines:

1. **Donor safety head:** donor-only versus the static cycle.  A repair is not
   allowed to retroactively certify a harmful donor.
2. **Semantic edge head:** donor plus shortcut versus donor-only, followed by a
   joint donor-plus-shortcut check versus the static cycle.

The frozen target vector retains mean E-OSPA, mean position RMSE, window and
terminal consensus, beneficiary and worst-formation metrics, worst-sensor
metrics, attempted-byte saving, and delivered-byte change.  A candidate is a
positive training label only when stage 1 is safe, stage 2 adds value, and the
joint action respects the risk and byte floors.

Truth and future outcomes score offline H=3 targets only.  Runtime inputs are
current graph, delivery history, local/fused posterior summaries, formation
need, exact credit, and scale-normalized geometry.  Dataset splitting, model
fitting, threshold selection, and calibration remain grouped by complete
scene-seed trajectory.

## Model and deployment projection

The state encoder remains permutation-equivariant and scale-aware:

- formation nodes summarize need, expected cardinality, label-existence
  dispersion, uncertainty, recent delivery reliability, and normalized
  geometry;
- donor nodes/edges encode removable full-posterior bytes and delayed-risk
  features;
- semantic edges encode source-to-beneficiary label support, compatibility,
  propagation delay, payload bytes, and credit utilization.

A small GNN predicts donor value and conditional semantic-edge value.  Ridge
heads over local/action features and pooled graph features remain mandatory
baselines.  The deterministic projector then applies physical reachability,
current delivery, complete-mixture payload, rolling recovery, exact-byte, and
calibrated lower-bound constraints.  If no action survives, it returns the
static cycle.

The GNN is promoted only if it improves unseen trajectory groups over both
ridge baselines.  Otherwise the simpler observable rule is retained.

## Comparison contract

V220 must answer four paired questions:

1. Does donor-only improve or preserve tracking while saving bytes versus the
   static cycle?
2. Does a cross-formation semantic shortcut add value over the paired
   donor-only state?
3. Does decoupling donor and beneficiary outperform the same-formation V216
   restriction under the same ledger?
4. Does targeted label routing outperform generic full-posterior mixing on
   tracking/cardinality, rather than merely reducing disagreement?

The paper-facing result is a communication--tracking Pareto comparison with
the static cycle, generic V27-style mixing, deterministic observable heuristic,
local/action ridge, graph ridge, GNN, and V206 teacher upper bound shown
separately.

## Immediate experiment order

1. finish and archive the X36 t=132 donor-only screen;
2. reconstruct a small cross-formation semantic bank at X36 t=118/t=132 from
   captured causal states;
3. evaluate no-op, donor-only, and donor-plus-shortcut H=3 arms before any
   larger trajectory collection;
4. require a material joint X36 gain, not merely an output-neutral byte saving;
5. only then collect grouped M24/X36 trajectories, fit the value heads, and
   freeze unseen-seed and richer-scene validation.

The current-best Lark tables remain V187 at strategy level and V206 at
mechanism level until a complete online V220 policy improves their frozen
comparison records.
