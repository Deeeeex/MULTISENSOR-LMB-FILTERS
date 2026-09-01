# V220 scale-aware semantic effective-graph routing

## Outcome update: residual shortcut rejected

The X36 seed-1301, t=132 H=3 screen completed after this design was frozen.
The communication-donor stage is retained: F6 improves E-OSPA, RMSE,
consensus, terminal consensus, and attempted bytes by 0.123%, 1.672%, 0.317%,
0.344%, and 0.430%; F5 is output-neutral while saving 0.574%.

The post-fusion semantic shortcut is rejected as the primary stage-2 action.
One F6-to-F1 candidate degrades its donor-only state, while the other adds only
about 0.001% across E-OSPA, RMSE, and consensus and leaves 0.008% byte saving.
An explicitly nondeployable synthetic-credit teacher for the previously
unaffordable top precision candidate improves F1 E-OSPA by 0.450% and network
consensus by 0.485%, but worsens F1 RMSE by 2.034% and network RMSE by 0.683%.

The successor therefore keeps the donor/credit mechanism but moves semantic
routing **inside the ordinary label-wise KLA input set**. A selected source
must replace or suppress a low-value/conflicting input, after which weights are
renormalized and mixture-aware KLA is executed once. It must not append a
second residual fusion after ordinary fusion. The detailed evidence and next
contract are recorded in
`V220_DECOUPLED_SEMANTIC_SHORTCUT_X36_T132_FINDING.md`. The remaining design
below is retained as the frozen rationale and audit trail; its residual-KLA
experiment order is no longer active.

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

## V221/V222 update: single-pass routing and query-first control

The first V221 X36-t132 H=3 screen resolves the post-fusion ambiguity.  S31
is not an ordinary F1 input in the static page, so the old failure cannot be
explained as literal duplicate insertion of the same source.  V221 instead
adds a complete label with zero global topology mass, reallocates weight only
for that label, and executes the ordinary mixture-aware KLA once.  Unrelated
labels and the static full-posterior backbone remain unchanged.

At a fixed source share of 0.10, the S7/[7,5] existence-deficit candidate
degrades E-OSPA, RMSE, consensus and terminal consensus by 0.172%, 3.226%,
0.494% and 0.409%.  The S9/[31,23] disagreement candidate is positive on all
four metrics, but only by 0.0002%--0.0008%, while preserving 0.131% attempted
byte saving.  This rejects uniform formation-wide source weighting as the
main action and shows that the V217 immediate risk proxy is not an H=3 action
value target.  No broad source-weight sweep or larger dataset is authorized.

The next control-plane change is query-first discovery.  V188 sends all
active local/fused label records from every target receiver and common source;
the t=132 F1 page therefore spends 12,300 B across 20 participants before a
label is requested.  All admissible V221 candidates already have receiver
support, so a beneficiary coordinator can first choose at most K receiver
labels, broadcast only their four-byte keys, and request risk-only records for
those keys from the remaining receivers and common sources.  With K=3, a
16-byte query header, four bytes per key, a 16-byte response header and 20
bytes per response record, the deterministic upper bound is

`(participants - 1) * [(16 + 4K) + (16 + 20K)]`.

For the current 20-participant page this is 1,976 B, reducing control discovery
by 10,324 B without omitting any field used to score the queried labels.  This
makes the S31/[13,12] complete payload affordable under the unchanged 20%
protected-credit reserve.  The first query-first run remains a frozen teacher
headroom test: it can establish whether the action space has multi-objective
value, but not whether an online coordinator can select the same label.

## V223 update: mixture-overlap safety projection

The query-first S31 teacher is communication-feasible but not tracking-safe.
It saves 0.227% attempted bytes and improves window consensus by 0.496%, while
degrading mean E-OSPA by 0.158%, mean RMSE by 3.169%, F1 E-OSPA by 0.914% and
F1 RMSE by 9.436%.  Source shares 0.05 and 0.10 yield almost identical
extracted tracks because both move the selected label below the 0.5 MAP
threshold.

Fusion-realized one-pass diagnostics rule out a routing-weight defect.  At R1, source
shares 0.05/0.10 produce `eta=0.244625/0.166985` and fused existence
`0.380350/0.386187`; at R3 they produce `eta=0.247875/0.169237` and existence
`0.422152/0.426163`.  The current V188/V190 scorer omits this term: it derives
compatibility only from existence, Bayes risk, position means and covariance
traces.  A candidate can therefore appear moment-compatible while its full
Gaussian mixtures have insufficient powered overlap.

The semantic action is now a receiver-specific edge in the label-wise
effective KLA graph.  The deterministic projector uses the Bernoulli
identity

`logit(r_fused) = sum_i omega_i logit(r_i) + log(eta)`

and admits an edge only when the powered-GM `log(eta)` produced by the same
fusion implementation retains the ordinary fused
existence and does not cross the MAP-cardinality threshold.  The executable
condition is

`log(eta) >= required_fused_log_odds - sum_i omega_i logit(r_i)`.

The log-odds identity is exact for the supplied normalizer; the normalizer is
still the repository's truncated componentwise powered-GM approximation, not
an exact arbitrary Gaussian-mixture power.  This projection is evaluated
after complete-payload delivery but before
recursive state mutation, so an unsafe receiver falls back to the ordinary
static input set.  Communication remains charged to the exact ledger even
when the payload is rejected.  The GNN is correspondingly demoted from safety
arbiter to finite-horizon value ranker over only the physically reachable,
byte-feasible and eta-safe edge set.  The pure gate contract is implemented by
`common/projectLabelKlaExistenceRetentionByEtaV223.m`; it must be integrated
and exercised on the frozen t=132/t=118 states before any broad weight sweep,
dataset expansion or M24 training.
