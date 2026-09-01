# V220 scale-aware edge-value routing

## Decision

V218/V219 remains useful as graph-state, trajectory-grouped dataset, and
finite-horizon value infrastructure.  Its primary action space is changed,
however.  Formation-wide posterior withholding is retained only as a negative
mechanism control.  The deployable method will choose a physically executable
cross-formation routing graph and will transmit complete Bernoulli Gaussian
mixtures on every selected edge.

The method is now:

1. use a permutation-equivariant formation graph to predict the three-step
   value of candidate information-flow graphs;
2. project every proposal onto physical reachability, strong connectivity,
   temporal recovery, and message-budget constraints; and
3. fall back to the registered static cycle when no proposal has a positive
   calibrated lower value.

The learned model ranks already executable graphs.  It does not create links,
relax the communication ledger, or certify connectivity.

## Key X36 finding

The X36 seed-1301 reference trajectory separates sensing geometry from routing
failure.

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
holds for 21 of 24 labels.  The failure is therefore not explained by a narrow
FoV, missing birth components, or a hard posterior-size cap.  Instead, label
existence is strongly localized: for some target groups one formation has
mean existence near one while another is near zero.

At both selected times every formation pair is physically reachable.  The
reference nevertheless schedules only six cross-formation directed messages
per page, forming the fixed cycle

`F1 -> F2 -> F3 -> F4 -> F5 -> F6 -> F1`.

Its formation diameter is five pages.  The t=130--131 delivery history also
loses one of two scheduled F3-to-F4 messages and both scheduled F5-to-F6
messages.  This is consistent with the observed information localization.
It is strong diagnostic evidence for a routing bottleneck, but the causal
tracking gain must still be established by paired route counterfactuals.

The completed t=118 withholding screen confirms that deleting a message is not
the main remedy.  Its only multi-objective eligible action improves E-OSPA,
RMSE, consensus, and attempted bytes by just 0.106%, 0.136%, 0.163%, and
0.240%, respectively.  These values remain negative-route evidence and do not
update the current-best method table.

## Executable action bank

The formation-level action bank contains complete graph alternatives rather
than isolated edge edits:

| Action | Cross-formation message count | Connectivity contract | Role |
|:--|--:|:--|:--|
| Static registered cycle | F | One directed Hamiltonian cycle | no-op baseline |
| Equal-budget cycle reroute | F | Another physically reachable directed Hamiltonian cycle | redistribute propagation delay without adding messages |
| Cycle plus one shortcut | F+1 | Registered cycle retained; shortcut physically reachable | buy one bounded reduction in information delay |
| Strong static graph | fixed larger budget | Frozen before outcome scoring | quality-matched communication reference |

An arbitrary one-edge replacement is not a valid atomic action: removing one
edge from a minimal directed cycle generally destroys strong connectivity.
Equal-budget rerouting therefore selects a complete alternative Hamiltonian
cycle, even when several directed edges differ from the registered cycle.

Candidate cycles are generated deterministically from the current physical
formation graph.  The bank is capped with identifier-free structural rules,
for example cycle diameter, recent delivered-path overlap, and source-to-deficit
delay.  The learned value model sees normalized graph and posterior summaries
only after this executable bank has been built.

## Three-step target

Every candidate graph is replayed from the same cached predecision state and
compared with no-op over three pages.  The target retains the existing V218
multi-objective convention:

- mean E-OSPA gain;
- mean position-RMSE gain;
- window and terminal consensus gain;
- target-formation and worst-formation gains;
- worst-sensor gains;
- attempted-byte saving and delivered-byte change.

Truth scores offline targets only.  Runtime features exclude truth, future
measurements, and future delivery outcomes.  Dataset splitting, model fitting,
and calibration remain grouped by complete scene-seed trajectory.

## Comparison contract

No single weak static graph is sufficient as a baseline.  V220 reports two
paired trade-off questions:

1. **Equal communication:** does a learned equal-budget cycle improve tracking
   and consensus over the registered static cycle with the same number of
   cross-formation messages?
2. **Equal quality:** does a learned cycle-plus-shortcut policy approach a
   frozen strong static graph while using fewer attempted bytes?

The paper-facing result is a communication--tracking Pareto curve, with static
cycle, strong static graph, deterministic observable heuristic, local/action
ridge, graph ridge, and trainable GNN shown separately.  A GNN is promoted only
if graph context improves unseen-trajectory results over both ridge baselines.

## Immediate experiment order

1. finish the already running X36 t=132 withholding batch and archive it;
2. generate the executable cycle and shortcut bank at X36 t=118/t=132;
3. run paired H=3 labels for a small bank before collecting more trajectories;
4. require at least one non-trivial equal-budget positive route action on X36;
5. only then collect M24/X36 training trajectories and fit the value model.

The current-best Lark tables remain V187 at strategy level and V206 at
mechanism level until a complete online route policy improves their frozen
comparison records.
