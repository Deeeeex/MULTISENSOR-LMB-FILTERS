# V192: cardinality stability is necessary structure, not a complete gate

## Result

V192 constructs the exact outcome-marginal cardinality PMF for the registered
full-payload reference and each one-formation omission counterfactual.  If the
reference MAP margin is `Delta` and the PMF infinity-norm perturbation is
`delta`, then `delta < Delta / 2` certifies that the MAP cardinality is
preserved.

The opened M24 t=104 state shows why this certificate cannot be used as a
standalone release rule:

| Formation | V99 selected | Ref -> candidate MAP at worst receiver | Minimum certificate slack | Recursive interpretation |
|:--|:--:|:--:|--:|:--|
| F1 | yes | 4 -> 13 | -0.905476 | strongly beneficial omission action |
| F3 | yes | 14 -> 14 | -0.054579 | beneficial and cardinality-preserving |
| F4 | yes | 9 -> 12 | -0.794171 | harmful over-cardinality / RMSE failure |

The desired F1 recovery and the harmful F4 over-count both change the
predicted MAP cardinality and both fail the sufficient certificate.  A rule
that releases every uncertified formation would therefore remove the main
source of M24 improvement together with the failure.

The X36 state reaches the same conclusion: the certificate fails for every
formation, while the predicted MAP changes for selected F2, F4 and F5 but not
F1.  Cardinality instability is informative, but not directionally
task-aware.

## Method decision

The next risk gate must distinguish supported recovery from unsupported set
entry.  For labels entering the candidate MAP extraction, it should combine:

1. the cardinality and top-label-set change;
2. current receiver measurement-association support;
3. current cross-formation support and useful-information loss;
4. a full-posterior fallback only for unsupported or ambiguous entries.

The MAP-margin certificate remains useful as a theorem and a continuous risk
feature.  It is not itself the online action selector.

## Evidence boundary

All V192 features are computed before joining any tracking outcome and use no
truth or future information.  The beneficial/harmful interpretation comes
from already-opened paired recursive M24/X36 development outcomes and is not a
validation or generalization claim.

