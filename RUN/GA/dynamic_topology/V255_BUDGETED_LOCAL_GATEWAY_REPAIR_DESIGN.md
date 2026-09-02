# V255 budgeted local gateway repair

## Decision from V252--V254

The action space is not the current bottleneck.  Under the frozen V252 grid,
seeds 1302, 1303 and 1304 contain respectively 3, 4 and 4 strict joint-positive
gateway windows.  V253 nevertheless produces no safe-positive selection on
seed 1304, and its predicted utility ranges from 4.353 to 51.065 while every
realized selected utility is non-positive.  V254 reduces the representation to
16 compact additive features, but both leave-one-seed-out folds still produce
zero safe-positive selections.  Five of its six holdout projections activate,
and all five assignments are absent from the teacher bank.

These observations reject two parts of the previous design rather than dynamic
gateway routing itself:

1. regressing the minimum of six gains turns whichever near-zero metric is
   smallest into a discontinuous label and discards the structure of the other
   five outcomes; and
2. maximizing an additive score over all formation arcs composes unseen global
   assignments even though the paired data primarily cover localized edits.

## Revised problem

V242 already supplies the main structural result: a strongly connected
formation backbone with exactly `N + 2(F-1)` posterior messages per page.  On
the corrected M24 episode it is 10.041% below the static-tree attempted bytes
and improves the network means, but its weakest-formation RMSE is 24.085%
worse.  The next method should therefore repair the localization tail while
preserving most of V242's communication headroom.  It need not prove that every
three-page repair is cheaper than an already sparse V242 page.

At a decision time, V255 either keeps V242 or replaces one directed physical
gateway `i -> j` by another physical edge with the same source and receiver
formation roles.  All other cross-formation gateways, the formation tree,
local cycles and KLA weights remain unchanged.  This is a bounded local repair,
not a new unconstrained graph-generation problem.

## Why one directed arc

The frozen 18-window V252 dataset was re-read with compact control charged once
per three-page hold from only the two formations involved in an edit.  If a
candidate must improve E-OSPA, RMSE and consistency, stay within the existing
2% formation-tail tolerance and may spend at most 2% above V242 in that local
window, a positive action exists in 13/18 windows.  Restricting the action to a
single changed directed arc still covers all 13 windows.  Under the original
zero-increment byte condition, a single-arc positive action remains available
in 10/18 windows.

Thus global composition adds extrapolation risk without adding positive-window
coverage in the available data.  A single-arc action also scales linearly in
the number of formation-tree arcs and quadratically only in the sensors of the
two incident formations.

## Communication accounting

V254 collected a 32-byte synopsis from all sensors and sent a full route command
on every page.  A single-arc V255 decision only needs the source and receiver
formations.  For M24, with six sensors per formation, one decision costs

`2 * 6 * 32 + 16 + 8 = 408 B`.

The chosen assignment then persists for the three-page horizon, so this cost is
paid once rather than three times.  X36 also uses six sensors per formation, so
the same local contract remains 408 B even though the formation count grows
from four to six.  The control scaling is therefore `O(S)` for sensors per
formation, not `O(N)` for the whole network.

The online budget is expressed as communication credit.  Let `B_static(t)` be
the causal payload estimate for the registered static route and `B_242(t)` the
estimate for the V242 backbone.  The available credit is

`C(t) = max(0, B_static(t) - B_242(t))`.

V255 may consume at most 20% of this credit, including control telemetry.  It
therefore retains at least 80% of V242's instantaneous estimated saving over
the static route.  Actual attempted bytes, rather than only estimates, remain
the reported end-to-end metric.

## Vector-valued prediction

For every single-arc candidate, the learner predicts eight coordinates:

- network E-OSPA gain;
- network position-RMSE gain;
- inter-formation consistency gain;
- total attempted-byte saving after control cost;
- minimum formation E-OSPA gain; and
- minimum formation RMSE gain;
- E-OSPA gain of the receiving formation; and
- RMSE gain of the receiving formation.

The first sentinel is a multi-output ridge model.  It uses changed-edge
features, the incumbent edge, source/receiver formation summaries and causal
route history.  Features are normalized by formation size and do not include
numeric sensor IDs, numeric formation IDs, target truth or future pages.

At deployment, a candidate is admissible only when conservative lower bounds
for network E-OSPA, network RMSE and consistency are positive, formation tails
remain above their registered tolerances, the receiving formation is predicted
to improve, and the deterministic communication-credit test passes.  Among
admissible actions, the policy maximizes the conservative receiving-formation
RMSE gain and otherwise keeps V242.  Predicting coordinates separately prevents
a tiny byte or consistency fluctuation from erasing a large and repeatable RMSE
signal during training.

## Theory interface

Topology and communication properties hold independently of prediction:

1. replacing one physical gateway by an edge with the same directed formation
   roles preserves the bidirectional formation tree and the local cycles;
2. the posterior-message count remains exactly `N + 2(F-1)`;
3. the credit projection retains the registered fraction of the static-route
   communication saving in the causal byte estimate; and
4. shared edge and formation functions are equivariant to sensor and formation
   relabeling.

For learned outcome constraints, let coordinate-wise prediction error be
bounded by `epsilon_k`.  Tightening each predicted constraint by
`epsilon_k` makes every accepted action feasible under that error event.  If
the RMSE-coordinate error is bounded by `epsilon_R`, maximizing the tightened
prediction has objective regret at most `2*epsilon_R` relative to the best
truly feasible candidate that retains the same safety margin.  Development
experiments may estimate these margins, but no finite-sample coverage claim is
made until enough independent calibration seeds exist.

## Frozen evidence order

1. Seeds 1302--1304 remain preliminary development data.  They justify the
   one-arc action boundary and document why coordinate-wise worst-seed ridge
   aggregation was rejected; their old full V250 banks are not relabelled as
   newly generated V255 data.
2. Seeds 1307--1310 are additional independent training realizations.  Each
   evaluates V242 plus at most two ranked one-arc replacements per directed
   formation-tree slot at the same six predeclared anchors.  Every non-reference
   arm is charged the local control payload once for the complete H=3 hold.
3. After the pooled expected-outcome representation, feature set and ridge
   regularization are frozen using training data only, seeds 1311--1312 are
   opened solely to calibrate one-sided uncertainty and the abstention margin.
4. Seed 1306 remains the once-only H=3 development holdout.  It is not used to
   choose features, regularization or calibration margins.
5. Seed 1305 remains untouched.  It is used for a complete M24 episode only if
   seed 1306 supports the frozen selector.
6. X36 starts only after the M24 model, margins, event trigger and communication
   credit are frozen.  No X36 outcome may tune them.
7. Crossing, merge-split and curved-corridor scenes follow only after M24 and
   X36 independently retain lower tracking error, better consistency and a
   material communication saving relative to the static route.

GNN escalation is not authorized by this design.  It becomes meaningful only
if the local action labels are stable, a transparent expected-outcome model
exposes a repeatable nonlinear residual, and additional independent seeds are
available.

## Why the first multi-output sentinel was rejected

The first V255 sentinel fitted one ridge member per seed 1302--1304 and used
the coordinate-wise minimum across the three predictions.  This abstained in
17/18 windows and retained only 0.006% aggregate RMSE gain while spending the
control charge.  The result does not show that local repair lacks value: the
paired teacher contains positive one-arc actions in 13/18 windows.  It shows
that three stochastic realizations cannot support a worst-seed decision rule.

The desired deployable quantity is the conditional expected outcome of an
observable action, not the exact H=3 outcome under one future measurement and
link-drop realization.  V255 therefore adds independent realizations of the
same registered scene and time grid.  Model fitting pools these realizations
to estimate expected coordinate outcomes; separate calibration seeds estimate
one-sided abstention margins.  This separation is fixed before any new outcome
is observed and replaces neither the paired static reference nor the final
complete-episode test.
