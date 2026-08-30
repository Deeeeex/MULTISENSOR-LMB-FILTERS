# V161 observable one-hop label-value gate

## Frozen method components

V159 proves that high-value content is one hop away.  V160 then shows that a
truth-free minimum-posterior-risk rule can choose a useful source without a
learned model.  V161 freezes that source rule and removes the remaining
privileged input: it is not handed the 35 V159 labels.

At each of the 36 registered X36 receiver-time cells, V161 enumerates every
active label held by a current physical neighbor.  For each label it chooses
the neighbor with minimum current per-label Bayes-risk proxy, then compares six
predeclared receiver-label policies built from:

- receiver minus source posterior-risk proxy;
- credible-source existence and same-label disagreement;
- a handover-rescue combination of existence, precision, association,
  observation opportunity and disagreement;
- two- versus four-label receiver budgets and a nonnegative-risk guard.

No policy reads target truth or future measurements.  Truth is used only after
selection to score immediate E-OSPA, and by a separate positive greedy oracle
over the same frozen one-hop sources.

## Communication accounting

The full Bernoulli GM payload is charged only for selected labels.  Candidate
discovery is not free: V161 also reports a conservative directed-message cost
for two synopsis contracts.

- Risk advertisement: 16-byte header plus 8 bytes per active label (label key
  and scalar risk).
- Rich synopsis: 16-byte header plus 64 bytes per active label, matching the
  compact moment/evidence contract used elsewhere in the repository.

The first contract is sufficient for the frozen source rule and pure
risk-reduction policy.  Richer policies must justify their larger metadata
cost.  These are preflight estimates; final recursive runs must put the chosen
contract into the filter's actual byte ledger.

## Decision rule

V161 passes the analytic-label route only if at least one observable policy
retains a substantial fraction of the same-source-set truth oracle while
avoiding harmful receiver-time cells.  If the oracle is strong but every
analytic policy has large false-positive or local-tail costs, source selection
is considered solved but label-value learning is justified.  If even the
oracle is weak, the one-hop replacement mechanism is closed before any model
is trained.

## Result and next gate

The frozen `risk-reduction-k4` policy is the only tested policy that satisfies
both development constraints.  It captures `66.315%` of the same-source-set
positive oracle, leaves every registered receiver-time cell beneficial
(`minimum cell gain = 12.104559`), and retains an estimated `+3.333%` byte
saving after charging the conservative risk advertisements and selected-label
responses.  This is sufficient evidence to prefer a small analytic rule over
a learned label-value model at the present stage.

The rich `handover-rescue-k4` policy captures `96.628%` of the oracle with no
harmful cells, but its synopsis overhead changes the adjusted communication
result to `-2.331%`.  It therefore remains a mechanism reference rather than a
deployable candidate unless a substantially smaller observable metadata
contract is derived later.

V162 must now run `risk-reduction-k4` recursively in the actual filter on the
same frozen 36 receiver-time cells.  The filter ledger must charge risk
advertisements, requests, response headers and complete-label payloads, and the
paired run must report E-OSPA, position RMSE, consistency and attempted bytes.
Passing V162 will justify a later V163 experiment that removes the last
privilege by detecting useful receiver-time cells online.
