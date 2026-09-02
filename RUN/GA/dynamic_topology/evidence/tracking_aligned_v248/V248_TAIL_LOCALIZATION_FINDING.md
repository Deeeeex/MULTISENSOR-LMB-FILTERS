# V248 weakest-formation tail localization

## Finding

The corrected M24 V248 result has a genuine metric conflict rather than a
uniform sparse-backbone failure.  The minimum causal backbone improves the
full-network mean E-OSPA, RMSE, consistency and attempted bytes over the
paired fixed tree, but its weakest-formation RMSE regresses by 24.085%.

The regression is concentrated in formation 4 during `t=58--73`:

| Window | Formation-4 E-OSPA difference | RMSE difference | Absolute-cardinality-error difference |
|:--|--:|--:|--:|
| `t=58--73` | `-6.275` | `+19.348` | `-1.323` |
| `t=70--72` | `-9.256` | `+18.328` | `-1.889` |
| peak `t=73` | `-11.115` | `+25.098` | not aggregated |

Negative E-OSPA and cardinality differences favor the minimum backbone;
positive RMSE differences favor the fixed tree.  Thus the sparse route keeps
more targets in the estimated set while making the matched target states less
accurate during the handover tail.  Network-average E-OSPA alone would hide
this failure mode.

## Method consequence

The registered V250 `t=70--72` continuation covers the severe part of this
tail and is therefore a relevant finite-horizon test.  A useful gateway action
must reduce formation-4 matched-state RMSE without giving back the existing
cardinality, E-OSPA, consistency or communication benefit.

If the V250 oracle establishes such an action, a deployable ranker must not be
limited to link reliability, distance or connectivity features.  Its causal
features need to represent localization complementarity, including per-label
sender/receiver uncertainty, spatial disagreement and receiver-side support.
These are ranking features only: the executed fusion remains mixture-aware
LMB-KLA, and no moment summary is treated as an equivalent replacement for
the LMB density.

## Evidence boundary

This is a post-result diagnostic on the single opened M24 seed 1301.  It
localizes the V248 tail and constrains the next method design; it is not an
independent validation result, a V250 gain, or an X36 claim.
