# V74 receiver direct task-risk finding

V74 does not provide a usable direct routing gate.  Every mixture-aware V73
candidate reduces the internal posterior Bayes-risk surrogate, but every
candidate also increases exact one-round network disagreement.  The frozen
conjunction therefore rejects every non-reference route on both opened
anchors.

| Anchor | Candidate formations | Affected Bayes objective | Mean disagreement gain | Tail disagreement gain | Direct gate |
|:--|:--|:--|--:|--:|:--:|
| M24, t=80 | `3` | `+0.0322772` | `-0.619%` | `-0.027%` | fail |
| X36, t=52 | `4` | `+0.0992041` | `-0.407%` | `-0.596%` | fail |
| X36, t=52 | `5` | `+0.0319276` | `-0.710%` | `-0.574%` | fail |
| X36, t=52 | `[4 5]` | `[+0.0992041,+0.0319276]` | `-1.103%` | `-1.107%` | fail |

The failure is methodological rather than a threshold issue.  The Bayes-risk
term scores each candidate under the candidate's own posterior.  A sharper or
more confident posterior can therefore receive a lower internal risk without
showing that its mean or label support is closer to the unknown state.  At the
formation-action level it does not distinguish the immediately harmful
X36 formation-5 intervention from the useful M24 formation-3 and X36
formation-4 interventions seen in the opened V72 tracking diagnosis.  V73
changed one exact sender slot at each scale and did not execute the aligned
routes, so the V72 outcomes are mechanism evidence rather than outcome labels
for the exact V73 candidates.

The disagreement constraints answer a different question.  They require the
candidate not to move the network farther apart after one fusion round.  V72
already showed that M24 formation 3 improves immediate E-OSPA while worsening
consensus, so making non-increasing disagreement a direct task gate rejects a
known accuracy--consensus tradeoff by construction.  Relaxing the sign or
tuning a tolerance would not repair the missing task signal.

V74 therefore closes two paths:

1. self-posterior Bayes risk is not accepted as a task-accuracy certificate;
2. one-round network disagreement is not accepted as a hard direct-utility
   constraint.

The next source-only diagnostic must measure the spatial conflict hidden by
the existence-only opportunity.  For every exact receiver--incumbent--new
sender replacement, V75 will compute a covariance-normalized innovation
energy over labels shared by the two senders and supported by the receiver.
Reference-relative label retention and cardinality change remain separate
safety terms.  The diagnostic must be applied to both the exact V72 routes,
whose opened time-resolved outcomes provide retrospective mechanism evidence,
and the exact V73 routes, which are scored prospectively and receive no
positive or negative tracking label.  Network disagreement remains a recovery
cost to be budgeted over time, not a zero-tolerance proxy for task accuracy.

This is opened-anchor development evidence only.  V74 executed no route, read
no truth or future measurement, and supports no tracking or generalization
claim.
