# V274 X36 minimum-backbone baseline

## Purpose

The current full-episode best method, V242, has a paired M24 result but no
tracking-aligned X36 comparison against a fixed route.  Further optimization is
not interpretable until that missing scale baseline is measured.  V274 therefore
adds no new routing rule.  It asks whether the existing causal minimum backbone
retains practical value when the network grows from four six-sensor formations
to six six-sensor formations.

## Frozen case

- scene: `x36-formation-fov-temporal-coupled-formation-braid`;
- seed: `1301`;
- length: the complete 160-step episode;
- fusion, measurements, target truth, communication realization and filter RNG:
  identical across paired arms;
- field of view and distance-aware sensing: inherited unchanged from the
  registered 120-degree temporal-coupled scene.

The scene preflight must pass before tracking starts.  No M24 threshold or
gateway parameter is retuned on X36.

## Arms

1. **Fixed formation tree.**  Formation pairs never reroute.  When a pair is no
   longer physically feasible, its unavailable messages disappear and their
   fusion mass returns to self.  This is the no-dynamic-routing baseline.
2. **Full causal repair.**  The preceding feasible formation tree is preserved;
   physical infeasibility triggers a minimum-edit repair.  Every receiver keeps
   two nonself inputs, for exactly `2N = 72` posterior messages per step.
3. **V242 causal minimum backbone.**  It uses the same causal formation-tree
   repair but keeps one directed local cycle per formation and only the two
   directed gateways per tree edge.  Its fixed architecture uses
   `N + 2(F-1) = 46` posterior messages per step.

The full causal arm isolates the value of dynamic repair; the difference from
full causal to V242 isolates the effect of removing redundant local inputs.

## Reported decision levels

The report keeps numerical results even when a paper threshold is missed.

- **Direction:** V242 improves full E-OSPA, full conditional set RMSE and focus
  consistency over the fixed route while saving attempted bytes.
- **Material X36 value:** at least 2% E-OSPA, 5% RMSE and 5% attempted-byte gain,
  nonnegative focus-consistency gain, and no formation worse by more than 1%
  E-OSPA or 5% RMSE.
- **Inherited paper threshold:** the stricter V242 5% E-OSPA, 5% RMSE, 2%
  consistency, positive communication saving and nonnegative formation tails.

These are three distinct statements; missing the strict threshold does not hide
the best numerical result.

## Evidence boundary

This is one opened X36 development seed.  It directly answers whether the
current method has cross-scale headroom, but it is not multiseed, multistyle or
validation evidence.  Failed or sub-threshold results stay in the experiment
record.  A positive result may update the current-best table, while a paper
claim still requires frozen independent seeds and additional scene styles.
