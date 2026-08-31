# V201 formation-label action-selector preflight

## Question

Can the remaining X36 repair problem be reduced to a single rule that sends
the current formation-label candidate with the largest observable Bernoulli
Bayes-risk reduction?

The answer is **no**.  Two independent checks show that the action must first
be classified by failure mechanism and only then ranked within that action
type.

## Receiver granularity is not identifiable in F3

The V200 current-neighbor action table contains all six F3 receivers at
`t=76`, `t=78`, and `t=79`.  After collapsing sources by semantic label, every
receiver at a given page selects the same label `[19,13]` with essentially the
same observable score:

| Time | Best source | Top risk reduction | Runner-up label | Dominance | Precision gain | Source quality |
|--:|--:|--:|--:|--:|--:|--:|
| 76 | 23 | 0.3269 | 0.1403--0.1706 | 0.478--0.571 | 0.999 | 0.916 |
| 78 | 23 | 0.3204 | 0.1644--0.1647 | 0.486--0.487 | 0.999 | 0.906 |
| 79 | 22 | 0.3171 | 0.1775--0.1932 | 0.391--0.440 | 0.999 | 0.904 |

Only sensors 15, 15, and 16 exhibit the large matched-RMSE spike, but no
receiver-local risk threshold separates them from their five formation peers.
The spike is created by MAP label-set selection, whereas the fused spatial
object for `[19,13]` is shared.  A deployable rule therefore cannot claim to
identify the individual bad receiver from these features.  The correct
mechanism experiment is formation-level multicast of one complete label,
with receiver-specific residual KLA outputs.

## Maximum risk does not identify the useful formation

The frozen V190 X36 `t=72` candidate bank contains several formation-common
actions with larger current risk than the F2 action already shown useful by
V199:

| Formation/action | Minimum risk reduction | Precision proxy | Source evidence | Source observation opportunity | Peer consensus | Receiver compatibility | Max distance / cutoff |
|:--|--:|--:|--:|--:|--:|--:|--:|
| F2, source 19, label `[13,12]` | 0.3144 | 0.3739 | 0.9991 | 0.8216 | 0.2561 | 0.0198 | 1.2204 |
| F4, source 18, label `[25,17]` | 0.4027 | 0.8094 | 0.7718 | 0.7121 | 0.1533 | 0.0000 | 3.2367 |
| F6, source 19, label `[7,5]` | 0.4987 | 30.6492 | 0.9984 | 0.7917 | 0.3824 | 0.8942 | 1.8576 |

A maximum-risk selector would choose F6, then F4, before F2.  F4 can be
rejected by its weaker source evidence and extreme spatial displacement, but
the F6 candidate remains strong under ordinary source-quality, opportunity,
peer-support, and compatibility checks.  There is currently no causal basis
for hard-coding a threshold that accepts F2 while rejecting F6.

The `precisionGainNormalized` value also is not a bounded normalized feature:
it divides a covariance-trace difference by the E-OSPA cutoff squared and can
greatly exceed one.  Future selection should use the bounded log trace-ratio
precision feature already exposed by
`computeObservableLmbLabelTransferFeatures`, while retaining the raw value
only as a diagnostic.

## Method consequence

The online action space should distinguish at least three semantic modes:

1. **Precision refresh:** receiver and source are spatially compatible, but
   the source is substantially more precise.  F6 is an unresolved candidate
   of this type and needs its own teacher evaluation rather than rule-based
   rejection.
2. **Observation handover rescue:** the receiver is currently blind or
   uncertain while a high-quality source currently observes the label.  The
   F3 `[19,13]` action is of this type even though receiver-source and peer
   spatial compatibility are near zero.
3. **Set/cardinality restore:** the defect is diffuse across labels or the MAP
   set and cannot be repaired by one spatial label update.  The M24 evidence
   retains full-formation restore for this mode.

No-op remains a fourth action.  A first deployable selector must classify the
mode from current, scale-normalized metadata and estimate finite-horizon value
within the selected mode.  A single handcrafted risk threshold is now closed.

## Current bounded experiment

The running V201 H=8 teacher combines the established F2 action at `t=72`
with formation-level F3 label-KLA actions at `t=76`, `t=78`, and `t=79`.
Page, formation, source, and label identifiers are teacher routing keys.  The
experiment tests recursive mechanism headroom only; it does not validate an
online selector.

## Evidence boundary

All selector features above come from already captured current posterior,
current physical-neighbor, geometry, and source metadata.  The F3 attribution
uses current truth only to explain the previously observed output error after
candidate construction.  No result here is cross-seed, held-out, deployable,
or suitable for the main progress document until the complete policy passes
the registered M24 and X36 gates.
