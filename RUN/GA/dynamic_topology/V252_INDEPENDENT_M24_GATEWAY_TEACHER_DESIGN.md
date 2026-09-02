# V252 independent M24 gateway-teacher dataset

## Decision being tested

V251 showed that the 47-feature posterior-rich pairwise ridge has enough
capacity to reproduce all three V250 tail-aware teachers in sample, while
leave-one-anchor-out transfer fails at every anchor.  Three windows from one
opened trajectory therefore cannot distinguish insufficient data from a
non-generalizable representation.  V252 adds independent scene realizations
before any GNN or complete-episode controller is authorized.

## Frozen sampling and split

The experiment uses the corrected M24 temporal task-coupled formation-braid
scene.  Anchor times are the fixed grid `t = 40:20:140`, giving six disjoint
H=3 windows per seed.  The grid is chosen before any posterior, truth or arm
outcome is opened; it covers early, middle and late phases without selecting
only high-risk events.

- seeds 1302 and 1303: ridge training and model selection;
- seed 1304: development holdout, opened only after the representation,
  pairwise target, lambda grid and selection rule are frozen; and
- seed 1305: no H=3 teacher generation, reserved for a later complete-episode
  M24 test.

The old seed-1301 anchors remain V250/V251 development evidence and are not
silently counted as independent V252 samples.

## Paired arm contract

Every window begins from the V242 causal reference posterior and two-page
route history.  Candidate 1 continues the ordinary dynamic V242 reference;
the remaining candidates change only the sensor endpoints of the same six
directed cross-formation messages.  All arms reuse the same measurements,
link-delivery uniforms and filter RNG and preserve 30 attempted messages,
physical reachability, strong connectivity and KLA row sums.  The action
persists for H=3 when feasible and otherwise falls back to V242.

The V250 bounded candidate bank is deliberately reused unchanged.  This
isolates the question to cross-seed learnability rather than changing the
action space and data distribution together.

## Model-selection and stopping rule

Only seeds 1302 and 1303 may choose the ridge lambda from
`[1e-6, 1e-4, 1e-2, 1, 100]`.  Selection is leave-one-seed-out and uses the
tail-aware teacher ranking, not direct truth-worst-formation prediction.  The
chosen representation, normalization, lambda and deterministic safety
fallback are then frozen before seed 1304 is scored.

Seed 1304 authorizes a complete-episode policy only if at least half of its
six selections improve the reference-worst formation and its aggregate
E-OSPA, RMSE, consistency and attempted bytes are all non-worse, with neither
formation-level E-OSPA nor RMSE worse by more than 2%.  Failure means revise
the causal representation or stop the learning route; it does not authorize
a higher-capacity GNN.  Passing only authorizes a frozen full-episode M24 test
on untouched seed 1305, followed by no-retuning X36 transfer.

## Evidence boundary

V252 is finite-window development evidence.  Even a passing seed-1304 result
is not a complete-episode gain, an X36 result or a paper claim.  The main Lark
current-best table remains the V248/V242 full-episode M24 record until a
deployable policy improves it under the same paired evaluation.
