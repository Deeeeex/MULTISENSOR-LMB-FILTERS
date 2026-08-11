# V112 design: finite-duration downstream-risk labels

## Decision question

V109--V111 show that source abstention creates real aggregate gain but also a
long-horizon downstream debt that local classification and fixed periodic
broadcast do not control. Before fitting a temporal GNN, V112 asks whether a
simple finite-duration recovery action space contains a strictly safe X36
choice.

## Candidate family

All candidates use the V105 formation schedule and explicit source abstention,
but stop the intervention after 3, 4, 5 or 6 pages. Every later page restores
the static full-payload route and fusion inputs for the entire network. This
varies only one causal variable: how long an altered protected state is allowed
to accumulate before global recovery.

The matched run records, for each duration:

- mean and per-page E-OSPA gain;
- every formation's full-window and per-page gain;
- worst downstream formation regret;
- F6 non-gateway terminal gain;
- attempted-byte saving, consensus changes and rolling-B3 safety.

Truth and future outcomes create offline labels only. Duration is not selected
online and no result supports validation or generalization claims.

## Gate

A useful upper-bound action must achieve at least 5% mean gain, at least 5%
gain on every page from t75 onward, nonnegative formation and F6 peer-terminal
gains, positive communication saving, positive consensus gains and the frozen
B3 reserve. If no duration passes, duration prediction alone is not a viable
learning target and the action space must expose the downstream influence cone.
