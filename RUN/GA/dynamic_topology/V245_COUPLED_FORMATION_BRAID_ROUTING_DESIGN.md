# V245 coupled formation-braid routing comparison

## Question

The original formation-braid scene breaks a fixed sparse route, but its
paired target streams do not cross the failed inter-module cuts.  V244 fixes
that confound without changing the sensor motion, communication radius,
120-degree/300 m sensing envelope, target count, or tracking algorithm.
V245 asks whether task--topology alignment makes causal route repair produce
a larger and more stable tracking benefit.

## Frozen comparison

- Scene: `m24-formation-fov-coupled-formation-braid`.
- Seed: `1301`; full `160`-step episode.
- Fixed-tree reference: the initial formation tree is retained and physically
  unavailable messages are omitted.
- Full causal repair: retain two inputs per receiver and minimally repair the
  formation tree only when it becomes infeasible.
- Minimum causal backbone: retain one directed local cycle per formation and
  two directed gateway messages per formation-tree edge, for
  `N + 2(F-1) = 30` messages per step at M24.
- All arms reuse the same measurements, delivery uniforms, filter seed,
  mixture-aware support-renormalized LMB-KLA, and full-posterior payload.

V241 and V242 implement the two frozen mechanisms.  V245 only registers their
application to the V244 scene and collates the three-arm result; it does not
tune either policy from coupled-scene tracking outcomes.

## Metrics and decision rule

The main comparison reports full-horizon E-OSPA and position RMSE, focus and
terminal inter-formation disagreement, attempted and delivered bytes, and the
worst formation and sensor tails.

The result determines the next method step:

1. If the minimum backbone improves all four network means over the fixed
   tree, it remains the balanced base method.  A materially larger E-OSPA
   gain than the original scene indicates that task coupling was the main
   limitation.
2. If full causal repair improves tracking but the minimum backbone does not,
   the sparse backbone has removed useful within-formation information; add a
   small number of information-valued residual edges.
3. If neither repaired arm improves tracking, topology validity alone is
   insufficient; move directly to posterior/visibility-aware residual-edge
   selection rather than further hand-designed rewiring.

The existing paper gate remains unchanged: at least 5% E-OSPA and RMSE gain,
2% focus-consistency gain, 1% attempted-byte saving, and no negative formation
tail.  Failing that gate does not hide a new current-best record from the main
document.

## Evidence boundary

This is one opened M24 development seed.  It can attribute the effect of task
coupling and select the next method family, but cannot establish held-out,
cross-scale, or paper-level generalization.
