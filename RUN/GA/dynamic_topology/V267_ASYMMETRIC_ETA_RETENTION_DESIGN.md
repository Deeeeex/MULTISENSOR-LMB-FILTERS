# V267 asymmetric eta retention

## Method decision

V266 proves that eta-projected label trust has substantial spatial headroom:
network RMSE improves by `13.076%` and F4 event RMSE by `34.773%`.  Its set
metric degrades because the symmetric `0.25` log-odds allowance is also
applied when the ordinary receiver label is already below the `0.5` MAP
threshold.  Several safe-by-that-rule actions therefore suppress an already
weak label and the absolute cardinality error grows with the E-OSPA loss.

V267 changes only this retention semantic.  For an ordinary MAP-positive
label, the candidate cannot cross `0.5` and may lose at most `0.25` log odds.
For an ordinary MAP-negative label, the candidate may increase support but
may not reduce log odds at all.  The V242 backbone, V265 route selection,
V266 source-share grid, full label payload charge and KLA operator remain
unchanged.

This asymmetry is causal and label-local.  It protects a weak hypothesis from
being erased by spatial disagreement without allowing the controller to make
it executable by fiat: any increase must still be produced by the ordinary
KLA equation, including its overlap normalizer.  No truth or future outcome
enters the decision.

## Frozen screen

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`.
- Continuation: paired V242 state at `t=57`, evaluated through `t=73`.
- MAP-positive drop allowance: `0.25` log odds, never below `0.5`.
- MAP-negative drop allowance: `0` log odds.
- Source-share grid: `[0.05, 0.025, 0.0125, 0.00625]`.
- Evidence: opened-event mechanism screen; compact distributed controller
  cost is not yet claimed.

If V267 retains material RMSE gain and removes the E-OSPA/cardinality loss, it
authorizes one full M24 episode.  Otherwise the current per-hop fusion action
family is closed and the next structural option is packet-level label routing
with source provenance and explicit age, not a learned ranker or threshold
sweep.
