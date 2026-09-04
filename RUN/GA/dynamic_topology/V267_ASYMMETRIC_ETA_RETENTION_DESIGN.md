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

## Paired result and action-family boundary

V267 improves network RMSE by `5.536%`, F4 event RMSE by `14.694%`, and
consistency by `0.058%` over V242.  Network and F4 event E-OSPA regress only
`0.041%` and `0.126%`, substantially less than V266, but mean absolute
cardinality error is still `0.222%` worse.  Nine messages are delivered and
only three are admitted (`t=57,59,65`).

All three admitted actions increase, rather than decrease, the relay's
immediate selected-label existence.  The cardinality/E-OSPA loss appears one
or more pages later, after that modified F3 posterior is propagated through
ordinary backbone KLA toward F4.  An immediate receiver guard therefore
cannot certify downstream label survival.  Tightening it further merely
returns toward V265 and removes spatial value.

This closes the current per-hop-fuse action family.  The next structural test
must keep the original donor label and timestamp intact at the relay, predict
it across the explicit one-page delay, and fuse it only at the intended F4
beneficiary.  Both physical hops and failed attempts must be charged.  If that
packet-level route cannot retain RMSE value without set degradation, the
selected F1 label is not a causal KLA-safe source and the route should be
abandoned rather than learned.
