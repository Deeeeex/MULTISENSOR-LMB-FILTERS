# Label-effective omission semantics v1

## Scope

This checkpoint implements the communication semantics required before an
output-aligned edge--label oracle can be evaluated.  It is not a tracking
method result and makes no M24/X36 gain claim.

## Three distinct source states

For one receiver, source and label, the fusion input now distinguishes:

| State | Wire content | Fusion treatment |
|:--|:--|:--|
| participate | complete Bernoulli Gaussian mixture | retain the registered source weight |
| deliberately omit | label identifier in the explicit omission envelope, no density for that label | set this source's per-label KLA weight to zero and renormalize |
| naturally absent | neither density nor omission identifier | apply the unchanged reference missing-label rule |

The previous restricted-whitelist hook remains available for historical
experiments, but it is not used as the new protocol because it conflates the
last two states.

## Empty-payload delivery

A successful selective message may omit every local label.  Its posterior
array is then empty, but the message is not a dropped neighbor: the runtime
keeps a registered control envelope as a current fusion input.  The envelope
contains a 16-byte fixed record and one 8-byte label identifier pair for each
deliberately omitted local label.  A full message with no omissions carries no
extra envelope.  The control bytes are added to attempted and delivered byte
accounting.

Selective receiver-safe execution already requires always-heavy message
opportunities and disables stale-neighbor fusion.  Consequently, an omission
decision applies only to the current fusion step and is not silently reused as
a future cached abstention.

## Implemented boundary

- `fuseLmbPosteriorsByLabel.m` consumes per-source explicit omission sets and
  rejects contradictory or unregistered metadata.
- `buildExternalReceiverLabelMessagePlan.m` emits complete participating
  mixtures, explicit omission envelopes and charged byte counts.
- `runEventTriggeredDistributedLmbFilter.m` preserves a delivered zero-payload
  envelope and forwards its omission set into label-wise fusion.
- `test_label_effective_omission_semantics.m` covers participating,
  deliberately omitted and naturally absent labels.
- `test_explicit_omission_message_plan.m` covers the all-label-omitted envelope,
  byte charge and planner-to-fusion handoff.

The existing V90 partial-label and dual-threshold filter smokes remain green.
The next admissible step is a privileged finite-horizon oracle scored on final
recursive tracking output.  No learned ranking model is opened before joint
M24/X36 oracle headroom is established.
