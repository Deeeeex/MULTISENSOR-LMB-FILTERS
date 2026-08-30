# V193: observation support separates recovery from over-counting

## Observable separation

V193 combines the V192 marginal LMB extraction with the current receiver
measurement-association mass of every label entering the candidate MAP set.
The resulting rule is formation- and time-index free: among formations that
V99 proposes to withhold, release the ordinary full posterior whenever the
candidate MAP extraction contains an entering label below the registered
positive-support threshold.

| Scale | Selected formation | Candidate MAP change | Entering labels | Unsupported entries | Maximum receiver risk | Opened recursive interpretation |
|:--|--:|:--:|--:|--:|--:|:--|
| M24 | F1 | yes | 9 | 0 | 0 | beneficial |
| M24 | F3 | no | 0 | 0 | 0 | beneficial |
| M24 | F4 | yes | 3 | 1 | 0.0549 | harmful RMSE over-count |
| X36 | F1 | no | 0 | 0 | 0 | approximately neutral RMSE |
| X36 | F2 | yes | 4 | 2 | 0.1227 | harmful RMSE gap |
| X36 | F4 | yes | 1 | 0 | 0 | beneficial |
| X36 | F5 | yes | 3 | 2 | 0.1206 | harmful RMSE gap |

The same rule therefore preserves M24 F1/F3 and X36 F1/F4 while identifying
exactly M24 F4 and X36 F2/F5.  Risky formations that were not selected by V99
remain on the ordinary full-posterior path and require no release action.

At the worst M24 F4 receiver, two entered labels have strong current support
(`0.7609`, `0.9737`) and one has zero support; the unsupported entry is the
set-level failure missed by the old 0.5 per-label crossing gate.  At the
worst X36 F2 and F5 receivers, two of the entering labels in each formation
have zero support.

## Method decision

This provides the deterministic safety projection required after V191:

1. V99 proposes communication-saving posterior omission using current
   positive-net value;
2. V192 constructs the candidate marginal LMB cardinality and MAP set;
3. V193 removes formations with unsupported candidate set entries from the
   omission set, restoring the existing full-posterior path;
4. only the projected set is executed and charged.

No risk-magnitude threshold is tuned on outcome labels: the decision uses the
pre-registered positive-support threshold and the existence of at least one
unsupported entering label.  The current analysis now authorizes paired
recursive release tests for X36 F2 and F5.  It does not yet prove that the
joint automatic policy improves tracking.

## Evidence boundary

The V193 feature uses current posterior/link counterfactuals and current
measurement-association metadata only.  Numeric label values align Bernoulli
components but are not features.  The beneficial/harmful column is joined
from already-opened paired outcomes after the feature is computed; the
separation remains development evidence, not validation.

