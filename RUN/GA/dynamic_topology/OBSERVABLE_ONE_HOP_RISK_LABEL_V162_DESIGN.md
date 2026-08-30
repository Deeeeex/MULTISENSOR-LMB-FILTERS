# V162 recursive observable one-hop risk-label transfer

## Question

V161 showed that a present-time analytic rule can recover useful label content
without truth or future measurements: choose the physically reachable source
with the lowest per-label posterior Bayes-risk proxy, then retain the four
labels with the largest positive receiver-minus-source risk reduction.  V162
asks whether the same rule remains beneficial after its selected labels are
inserted into the recursive filter and every protocol byte is charged.

## Frozen paired experiment

- Scenario: X36 formation-FoV seed 211, opened return window `t=72:79`.
- Baseline: static full-payload LMB fusion, rerun in the same paired screen.
- Candidate backbone: the frozen V105 route, fusion weights and payload
  protection schedule.
- Trigger schedule: the 36 receiver-time cells inherited from the privileged
  V126 diagnosis: no trigger at `t=72:75`, formation 2 at `t=76`, formation 1
  at `t=77`, and formations 1 and 6 at `t=78:79`.
- Remaining privilege: only those trigger cells are preselected.  Sources,
  labels and payloads are chosen causally at run time.

The trigger schedule makes V162 development evidence, not an online method or
generalization result.  If V162 passes, V163 must replace it with an observable
online trigger.

## Causal action at one receiver-time cell

1. Each current physical neighbor advertises its active label keys and one
   scalar Bayes-risk proxy per label.
2. For every advertised label, the receiver chooses the delivered one-hop
   source with minimum risk.
3. It computes `receiver risk - source risk`, keeps only positive values and
   requests the largest four.
4. A successful request returns the complete Bernoulli Gaussian-mixture
   density for each selected label; the receiver replaces the corresponding
   post-fusion label before state extraction.

The action reads current local posteriors, current physical adjacency and the
frozen present-link delivery realization.  It does not read truth, future
measurements, a static alternative arm or a learned predictor.

## Delivery and communication ledger

For a fixed directed link and time, synopsis and response reuse the same
frozen forward-link realization; the request uses the frozen reverse-link
realization.  No new random draw is introduced.  The filter's main ledger
charges:

- synopsis: 16-byte header plus 8 bytes per advertised active label;
- request: 16-byte header plus 8 bytes per requested label;
- response: the existing heavy-payload estimator, including its header and
  every Gaussian-mixture component of each complete label.

These bytes are added directly to attempted and delivered payload matrices,
including physical links absent from the candidate's base fusion route.  They
must not be added a second time as an auxiliary payload estimate.

## Predeclared development gate

V162 passes only if the paired recursive result simultaneously satisfies:

- mean E-OSPA improvement at least 5%, with the inherited nonnegative
  worst-sensor, formation and formation-time checks;
- mean position RMSE improvement at least 2%, with nonnegative worst-sensor
  and minimum-formation RMSE improvements;
- nonnegative attempted-byte saving after all V162 messages;
- nonnegative window and terminal consensus improvements;
- the inherited post-maturity and target-peer safety checks.

A failure remains an experiment record.  A pass advances only to removal of
the privileged trigger schedule; it does not yet establish M24/X36
generalization.
