# V76 reference-recovery replay finding

The frozen V76 certificate fails for every historical and aligned route.  The
mean and upper-quartile candidate--reference node gaps increase over the two
fixed-reference recovery rounds:

| Route | Mean arm gap by round | Tail arm gap by round | Frozen certificate |
|:--|:--|:--|:--:|
| M24 V71/V72 | `0.014856, 0.016916, 0.025933` | `0.059424, 0.067663, 0.102718` | fail |
| M24 V73 | `0.010012, 0.013071, 0.026630` | `0.040048, 0.052285, 0.105509` | fail |
| X36 V71/V72 f4 | `0.003927, 0.004330, 0.009818` | `0.015709, 0.017319, 0.039274` | fail |
| X36 V73 f4 | `0.004250, 0.004436, 0.009836` | `0.016999, 0.017744, 0.039343` | fail |

This does **not** establish that the candidate routes are recovery-unsafe.  A
candidate--reference gap treats the reference posterior as the state to which
the candidate should return.  Useful information injected at one receiver is
supposed to spread to other nodes, so more nodes can differ from the reference
even while the localized perturbation is mixing correctly.

The same replay contains direct evidence of this ambiguity.  The maximum
single-node arm gap decreases from `0.223676` to `0.159503` on M24 and from
`0.101726` to `0.079256` on X36, while the mean and tail arm gaps increase.
Thus the perturbation peak shrinks as its support spreads.  Candidate internal
network disagreement relative to the reference also changes sign across
rounds.  For aligned X36, terminal mean disagreement is only `+0.001345`
worse while terminal tail disagreement is `-0.004173` better.  These numbers
do not support a one-sided route rejection.

V76 therefore falsifies its proposed recovery reference frame: contraction
toward the reference arm is not a valid certificate for whether newly injected
information reaches network consensus.  The next recovery diagnostic must
decompose the candidate--reference perturbation into a common network mode and
a disagreement mode.  The common mode represents information that has spread
through the network and may persist; only the centered disagreement component
should be required to contract under the reference mixing graph.

The replay remains source-only and useful as a mechanism result.  It confirms
that a one-step local action can become a distributed network perturbation even
after the topology returns to the reference graph.  It does not authorize
tracking, reject the V75-safe routes, or establish closed-loop recovery.
