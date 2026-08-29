# V152: safe graph-codebook value routing

## Decision object

V152 returns to dynamic KLA routing, but it does not ask a learned model to
emit edges.  At each decision state, a deterministic projector constructs a
small codebook of complete directed fusion graphs.  A later value model may
only rank those already-feasible graphs or abstain to a static reference.

This differs from the closed V56--V59 local-action line.  Those experiments
changed one formation, or composed independently valued formation actions,
and did not provide enough network-scale headroom.  V152 treats the complete
graph and its recursively generated posterior trajectory as the indivisible
action.  It also differs from V150--V151: every selected edge transmits a
complete mixture-aware LMB posterior; there is no per-label omission,
moment-only payload, or zero-payload shortcut.

## Frozen candidate codebook

The first headroom screen contains eight causal graph policies:

1. fixed clockwise residual cycle;
2. fixed counter-clockwise residual cycle; and
3. six diversity-regularized graph ranks produced from the pinned,
   truth-free label-set message-passing score.

Every rank is projected independently at every live page.  The projector
preserves physical reachability, the fixed dominant within-formation
backbone, row-stochastic KLA weights, the exact directed-message count and
the registered rolling three-page sensor/formation information-flow
condition.  The six ranks differ only in which physically feasible
cross-formation residual edges are used; all use the same dominant and
residual weights.  The frozen static graphs are included in the codebook, so
the oracle can explicitly fall back rather than manufacture a dynamic gain.

The existing M24 model supplies only a scale-shared observable score used to
generate diverse proposals.  Its score is not interpreted as an X36 value
estimate, and no transfer claim is made.  The headroom oracle scores the
resulting executable trajectories after the fact.

## Output-aligned oracle

The pilot starts from paired continuation caches and executes each codebook
policy over eight pages with common measurements, delivery uniforms and
filter random numbers.  A candidate is admissible relative to the better
static orientation only when all of the following hold over the complete
window:

- network mean E-OSPA is lower;
- worst-sensor and minimum-formation E-OSPA do not regress;
- MAP-set consensus does not regress;
- attempted bytes do not increase; and
- all selected-graph physical, message-count and rolling-connectivity checks
  pass.

The privileged oracle selects the lowest-E-OSPA admissible codebook policy,
or the better static orientation if none is admissible.  It uses truth and
future outcomes only to measure action-space headroom; those quantities are
forbidden to the eventual deployed selector.

The codebook passes the pre-learning gate only if M24 and X36 independently
reach at least 5% aggregate oracle gain, at least four of five opened
development seeds contain a strictly beneficial admissible dynamic action,
and the selected aggregate tail, consensus and communication metrics remain
nonnegative.  M24 cannot compensate for X36.  The first run is one opened
seed per scale; failure to produce any material action stops the full matrix.

## Deployable model, only after oracle success

If the cross-scale oracle passes, the learned component is a graph-level
relative value model, not an edge-imitation network.  It receives the current
observable formation graph, posterior summaries, link state, recent route
history, and a candidate-versus-reference graph difference.  It predicts an
H-step value vector containing mean tracking, sensor/formation tail,
consensus, full-horizon bytes and posterior-complexity debt.  Training uses
paired common-random-number rollouts with whole-scene and whole-seed splits.

At runtime, deterministic projection builds the codebook first.  The model
ranks complete feasible candidates, and a calibrated lower confidence bound
must be positive for every registered objective before a dynamic graph can be
executed.  Otherwise the controller uses the frozen static reference.  This
separates structural feasibility from empirical tracking value and avoids the
zero proposal-capture failure of earlier exact-graph imitation.

## Theory target

Feasibility is deterministic because every executable action is returned by
the exact projector.  Let \(V(a)\) be the finite-horizon value of a codebook
action relative to the static reference and \(L(a)\) a jointly calibrated
lower bound.  On the event that all candidate bounds cover their values,
selecting only actions with nonnegative componentwise bounds and otherwise
falling back gives nonnegative registered value.  With simultaneous coverage
at least \(1-\alpha\), the probability of violating that conditional safety
statement is at most \(\alpha\), plus measured simulation-to-deployment and
receding-horizon mismatch.  This is a conditional finite-codebook guarantee,
not an unconditional theorem that tracking error always decreases.

## Evidence boundary

The original M24 three-arm retrospective envelope and old X36 clean-scale
v9 result motivate this screen but do not count as V152 evidence.  The old
formation-FoV v9 policy was always on and failed its frozen cross-scale gate;
V152 must show new headroom from explicit safe alternatives and fallback.
Below-gate pilots remain repository-only.  No GNN training, reserved-seed
validation, richer-scene claim or main-document result is authorized before
the M24/X36 oracle gate passes.
