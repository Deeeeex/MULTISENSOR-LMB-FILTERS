# Structured set-proposal preregistration: M24

## Development question

The previous four-head relational MLP fits every one of the 54 training
states but captures only 2/54 states under leave-one-seed-out (LOSO)
evaluation. This experiment tests a specific alternative explanation:
teacher codes 90/91/92 do not define stable cross-state classes, and raw
absolute feature components encourage seed-specific fits. The registered
alternative predicts an unordered set of complete safe graphs from
relative observable features.

This is a development architecture test. It is not an M24 effect estimate,
an unseen-seed validation, or X36 evidence.

## Frozen data boundary

- Scenario: `m24-hard`.
- Design seeds: `[7,11,17,19,23,27,29,31,37]`.
- Predecision times: `75:83`, giving 81 state blocks.
- Executed state-collection policy: truth-free posterior-analytic action 80.
- Inputs: receiver-relative, formation-pair-relative and block-relative
  standardizations of the frozen 42 observable edge features.
- Offline labels: 3--4 current-truth, repair-free, exact rolling-\(B=3\)
  safe graphs per state.
- No feature, behavior action or selected history reads truth or future
  return. Teacher graphs are privileged development labels and are not
  deployable.
- The six-seed v1 artifact is immutable. Seeds 27/31/37 are added as
  independently hashed shards and merged by exact seed-time identity.

## Frozen model and candidate objective

- Four exchangeable MLP edge-score heads.
- Hidden widths: `[16,32]`.
- Weight decays: `[1e-4,1e-2]`.
- 200 full-batch Adam epochs, learning rate `0.01`.
- Target-to-head assignment is recomputed by the minimum set-softmax loss
  subject to one target per head and one head per target.
- A complete graph receives the sum of its selected edge logits.
- Each training-state softmax bank contains all teacher targets, the
  truth-free behavior graph and the deduplicated exact projections of 24
  frozen random relative-feature score directions.
- Deployment proposals never insert a teacher graph directly. Each head is
  passed through the unchanged exact rolling-\(B=3\) projector; excluding
  at most one selected edge per head yields at most 16 proposals.

## Result-blind selection and gates

Hyperparameters are ordered only by LOSO M24 metrics:

1. minimum per-seed state capture;
2. aggregate state capture;
3. target-graph recall;
4. mean best target-edge F1;
5. projection failures and then model complexity.

The selected configuration is refit in each LOSO fold with the same
registered initialization rule. It must simultaneously satisfy:

- at least 80% top-16 state capture over all 81 expanded states;
- at least 2/3 state capture on every design seed;
- at least 80% state capture on the previously frozen 10 value-bearing
  states, which cannot select the hyperparameters.

Passing all three gates authorizes only paired H=3 return generation for
the frozen proposed graphs. It does not authorize critic training, an M24
tracking-benefit claim, or any X36 run. Failure keeps all three blocked;
the gates must not be lowered after inspecting the result.

## Reproduction

After the v2 dataset and its SHA-256 are frozen in
`getRollingSafeStructuredProposalProtocol`:

```matlab
addpath(genpath(pwd));
[modelPath, result] = trainStructuredSetProposalModel();
```

The official run must start from a clean tracked source tree with no
untracked MATLAB/Octave source files.
