# V54 runtime, oracle, and learned-utility gate

## Runtime integration decision

V54 should be a feature-gated extension of
`runEventTriggeredDistributedLmbFilter`, not a copied filter implementation.
The flag is off for every existing arm, preserving the frozen V46--V53 path.
When enabled, it changes exactly three stages.

1. **Local update:** call
   `updateLmbWithSensorMeasurementAndEvidence` so each predicted label records
   current observation opportunity and evidence before any communication.
2. **Message construction:** on the same attempted edge opportunities as V46,
   charge and transmit a control synopsis first, then select full GM label
   payloads under the remaining byte budget. The oracle arm may inspect full
   posteriors only to produce a development upper bound and teacher targets;
   it must be marked nondeployable.
3. **Receiver fusion:** fuse only the delivered selected label objects, then
   run `projectSelectedLmbLabelFusionRetention` before finalizing the receiver
   posterior. Bytes rejected after receipt remain charged.

The V46 dominant cycle remains unchanged. V54 acts only on residual
cross-formation label payloads in its first implementation, so the new method
does not need a second connectivity mechanism.

## Stage A: oracle headroom before learning

The first V54 experiment is a paired X36 convoy development run using the
same scene, seed, delivery trace, filter seed, message opportunity set, and
V46 reference already frozen for V51--V53. It uses full posterior access only
inside the option teacher. Synopsis bytes are still charged because the later
online policy cannot receive features for free.

The oracle advances only if all conditions hold:

| Metric | Development requirement |
|:--|:--|
| Full-horizon position E-OSPA | at least 2.0% better than V46 |
| Mean absolute cardinality error | at least 2.0% better than V46 |
| Focus-window position E-OSPA | no worse than 0.5% |
| Total attempted bytes | no more than V46, including synopsis bytes |
| Post-receipt retention | zero unresolved existence-floor violations |

The 2% headroom requirement is deliberately stronger than the final 1%
online target: a learned approximation, control traffic, and conservative
fallback will consume part of the oracle gain. If the oracle fails, V54's
receiver-safe hypothesis has not created enough practical headroom and no GNN
training is justified.

## Stage B: synopsis-conditioned set GNN

The communication graph seen by one receiver-label is a small star: one
receiver and a bounded set of candidate senders. The learned model should
therefore be a permutation-invariant set GNN rather than an index-specific
edge scorer.

For sender `i`, an encoder consumes receiver synopsis features, sender
synopsis features, their moment disagreement, link reliability/age, and the
candidate subset indicator. Sender embeddings are summed and max-pooled;
the decoder predicts two option-level outputs:

- `log(1 + reference-to-candidate distortion)`;
- probability that the option will violate the receiver retention floor.

The model predicts a sender subset jointly. It does not sum independent edge
values, because same-label sender effects interact through KLA normalization.
The exact byte projection consumes the predicted option table and still
enforces the hard communication budget. The post-receipt projection remains
the final retention guarantee.

Allowed online features are limited to:

- existence and predicted-to-updated existence change;
- posterior mean and covariance synopsis;
- component count and normalized mixture entropy;
- observation opportunity, in-FoV probability, detection-association mass,
  and evidence type;
- link reliability, message age, formation relation, and byte cost;
- recent attempted/delivered traffic history.

Target truth, future measurements, future delivery outcomes, and unsent GM
components are forbidden. Training targets may use the offline oracle but
the exported model input contract may not.

The loss combines Huber regression on log distortion, binary retention-risk
loss, and an within-label option-ranking loss. Evaluation reports oracle
regret and selected-action agreement in addition to tracking metrics; raw
edge-classification accuracy is not sufficient.

## Stage C: generalization and final gate

Two forms of generalization are required.

1. **Scale transfer:** train the first model on M24 development traces and
   evaluate zero-shot on X36, then compare with a model allowed X36
   development data.
2. **Scene-family holdout:** hold out merge-split or curved-corridor as a
   complete family rather than splitting adjacent time steps from the same
   trajectory across train and test.

The online arm is retained only if it recovers at least 70% of the oracle
improvement, improves both full-horizon E-OSPA and cardinality by at least 1%
on M24 and X36, uses no more attempted bytes than V46, and has no scene-family
mean focus-window degradation above 1%. Final evidence must cover radial,
convoy, merge-split, and curved-corridor scenes with disjoint seeds.

## Implementation order after V53

1. Add the feature-gated three-stage runtime hooks and separate synopsis/GM
   byte ledgers.
2. Run a short execution smoke to measure active-label counts and selector
   runtime, then the single frozen X36 oracle development case.
3. Stop if the oracle gate fails; otherwise export teacher rows from M24 and
   X36 development scenes.
4. Train the set GNN in Python, export fixed weights, and run the online arm
   through the same Octave tracker.
5. Freeze model and thresholds before cross-scene and held-out-seed runs.
