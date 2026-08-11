# V110 design: opened safe-formation oracle

## Decision question

V109 proves that V105 already acts as source-level abstention. Its aggregate
X36 gain is real, but the action harms F1 and F6. Before adding a label-wise
fusion layer or training a GNN, V110 asks whether the simpler formation-level
action space has sufficient headroom when formation risk is classified
perfectly.

## Frozen oracle

V110 uses the opened V105 formation outcomes to exclude F1 and F6 from
protection at every page. It retains V105/V109 protection for F2--F5, the four
formations with positive full-window gains. The matched static route, explicit
abstention semantics, measurements, delivery uniforms, filter RNG and
communication accounting remain unchanged.

The resulting protection schedule is:

| Time | Protected formations | Always full-payload formations |
|--:|:--|:--|
| 72 | F2, F4, F5 | F1, F3, F6 |
| 73--79 | F2, F3, F4, F5 | F1, F6 |

This is an outcome oracle, not a deployable policy. A positive result would
show that a learned, observable formation-risk gate is worth pursuing before a
more complex label-wise controller. A failure would show that interactions
between formations make binary formation decisions insufficient even with
perfect retrospective classification.

## Gate

The oracle must preserve at least the inherited 5% mean E-OSPA gain, positive
byte saving and consensus improvements while making all formation gains and
the F6 non-gateway terminal gain nonnegative. Generalization remains unclaimed.
