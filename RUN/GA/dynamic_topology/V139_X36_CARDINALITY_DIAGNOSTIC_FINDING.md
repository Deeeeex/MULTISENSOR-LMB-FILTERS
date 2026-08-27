# V139 X36 diagnostic: a cardinality loss hidden by positive spatial gains

This is repository-only mechanism evidence.  The frozen V139 X36 screen was
replayed from commit `64fe7d77e73b2223cf521f921e79281c9e377d35` with output
cardinalities recorded at the local, working, exact-relay, label-readout and
final-output stages.  The replay reproduced the registered result exactly:
`+4.901%` intervention E-OSPA gain, `-0.161%` minimum sensor gain,
`+0.234%` minimum formation gain and `+89.403%` attempted bytes.  V139 still
fails its joint-scale gate and is not a communication method.

## Decisive attribution

All six severe cells (gain no greater than `-5%`) have the same discrete
failure:

| Sensor | Page | Gain | Local | W | R | Readout/final | Truth | R card. | Hybrid card. | R loc. | Hybrid loc. |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4 | 4 | -9.319% | 18 | 18 | 19 | 18 | 24 | 68.465 | 75.000 | 11.317 | 11.399 |
| 5 | 4 | -9.238% | 18 | 18 | 19 | 18 | 24 | 68.465 | 75.000 | 11.120 | 10.775 |
| 6 | 4 | -9.286% | 18 | 18 | 19 | 18 | 24 | 68.465 | 75.000 | 11.450 | 11.405 |
| 3 | 5 | -9.151% | 18 | 18 | 19 | 18 | 24 | 68.465 | 75.000 | 11.249 | 10.508 |
| 4 | 5 | -9.114% | 18 | 18 | 19 | 18 | 24 | 68.465 | 75.000 | 11.336 | 10.436 |
| 5 | 5 | -9.224% | 18 | 18 | 19 | 18 | 24 | 68.465 | 75.000 | 11.439 | 11.103 |

The X36 truth cardinality is 24 throughout this continuation.  Every severe
cell therefore changes the OSPA cardinality component from `68.465` under R
to `75.000` under the hybrid output.  The localization component is almost
unchanged and is better under the hybrid output in five of the six cells.
Thus the roughly nine-percent regressions are not spatial-fusion failures:
they are one-target extraction losses at a nonlinear cardinality boundary.

Across the ten intervention pages there are 23 materially negative cells,
of which eight have an R/final cardinality fork.  All six severe cells are in
that set.  There are also 100 R/final cardinality forks in total, and 91 occur
in cells where V139 beats R.  Cardinality disagreement is therefore a precise
explanation of the severe tail, but not a valid trigger for discarding the
whole hybrid output.

## Rejected repair and next method

Replacing the whole output by R whenever its cardinality differs from R is
decisively too coarse.  The frozen result-only counterfactual reduces the
intervention gain from `+4.901%` to `+0.021%`, with `-0.003%` minimum sensor
gain and `-0.001%` minimum formation gain.  It erases the useful W spatial
estimate at 91 positive fork cells.

The next bounded method is therefore a reference-cardinality-constrained
hybrid readout.  It retains the W/R per-label spatial choices but extracts
exactly the R MAP cardinality using the largest hybrid existence
probabilities.  This preserves the hybrid localization information while
equalizing the OSPA cardinality term with R.  Hidden W and R states remain
unchanged.  No threshold or margin sweep is authorized.
