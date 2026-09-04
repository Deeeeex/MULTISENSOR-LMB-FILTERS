# V266 eta-projected label trust

## Method decision

V265 shows that one complete label can improve the opened F4 localization
event without changing the full-posterior backbone.  It also reveals why a
fixed label share is brittle: the selected remote copy can be highly certain
yet spatially inconsistent with the relay.  In Bernoulli KLA that mismatch
appears through the spatial-overlap normalizer `eta` and can sharply reduce
the fused existence probability.

V266 keeps the V242 topology and the V265 causal label, formation and carrier
selection.  It does not relax the existence-retention envelope.  Instead the
receiver evaluates the frozen descending source-share grid
`[0.05, 0.025, 0.0125, 0.00625]` inside the same ordinary label-wise KLA and
uses the largest positive share that satisfies the envelope.  Thus `eta`
becomes a deterministic trust allocator: compatible information keeps the
full share, while inconsistent information is attenuated rather than either
forced into the estimate or discarded completely.

## Frozen screen

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`.
- Continuation: paired V242 state at `t=57`, evaluated through `t=73`.
- Full-posterior messages: exactly `N+2(F-1)=30` on every page.
- Added communication: one complete selected label per attempted physical
  hop, charged independently of delivery and selected KLA share.
- Safety: unchanged MAP threshold `0.5` and reference log-odds envelope
  `[-0.25,+0.25]`.
- Evidence: opened-event mechanism screen only; the centralized risk synopsis
  is not yet included in communication cost.

The grid is part of one online receiver projection, not four outcome arms.
Only the final recursive tracking trajectory is scored.  If this projection
does not preserve the V265 localization headroom while repairing its set and
tail metrics, the next decision moves to source value or multi-hop latency;
no learned ranker is authorized.

## Paired result and revised decision

V266 applies every delivered label message by selecting source shares between
`0.00625` and `0.05`.  Relative to the paired V242 window, network RMSE improves
by `13.076%` and F4 event RMSE by `34.773%`; these are substantially larger than
V265's `4.660%` and `12.353%`.  The action therefore has real recursive
localization value, and the eta-projected trust rule is the first deterministic
mechanism with material headroom on this event.

The joint gate still fails: network E-OSPA and consistency regress by `0.193%`
and `0.328%`, F4 event E-OSPA regresses by `0.751%`, and mean absolute
cardinality error increases by `0.690%`.  The degradation grows after the
projection admits several candidates whose selected-label log odds decrease
while the ordinary receiver is already below the `0.5` MAP threshold.  This
isolates the remaining issue to the existence side of the selected Bernoulli,
not to insufficient spatial influence.

The next variant must remain a standard label-wise KLA and use an asymmetric
retention rule: a MAP-negative ordinary label may gain support but may not lose
additional log odds, while a MAP-positive label retains the existing threshold
and `0.25` drop allowance.  This follows the intended safety semantics and is
not a threshold sweep.  If that rule cannot keep the localization gain while
repairing E-OSPA/cardinality, only then should the method move to source value
or packet-level multi-hop routing.
