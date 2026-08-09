# Braided-handover scale scene

## Why this scene exists

The radial and merge-split scenes are useful stress tests, but their larger
versions also expose each target to more formations.  That makes scale and
redundant visibility change together.  The braided-handover scene instead
repeats one local handover module: X36 is longer than M24, not locally denser.
It therefore tests whether a routing decision survives additional unrelated
formations.

## Geometry contract

- M24 uses four six-sensor formations; X36 uses six.  Formation centres form
  one moving chain, spaced by 180 m and sharing the same forward heading.
- Every sensor keeps the registered 120-degree FoV, 300 m range, and 35 m
  formation ring.  A 270 m communication radius admits adjacent modules and
  excludes two-hop modules.
- Adjacent formation pairs exchange two target groups in opposite
  longitudinal directions.  The groups travel in service lanes at `y=+110 m`
  and `y=-110 m`; they do not pass through a sensor ring.
- “Braided” refers to the two groups exchanging longitudinal ownership in the
  space-time diagram, not to targets weaving laterally through the sensors.
- Each target is intended to be seen by one formation most of the time and by
  two formations only during a short handover interval.  Target and formation
  counts grow together, while clutter density and local geometry stay fixed.
- Correlated link blockages remain causal and are attached to registered
  adjacent physical pairs.  The scene does not use posterior values, truth,
  future outcomes, or tracking results to define its geometry.

## Pre-generation analytic check

Using formation centres and all four within-group cross-track offsets, the
deterministic waypoint geometry gives closely matched visibility distributions:
M24/X36 mean visible formations are `1.1602/1.1729`,
zero-formation fractions are both `0`, single-formation fractions are
`0.8398/0.8271`, and multi-formation fractions are `0.1602/0.1729`.
The minimum centre-to-target distance is `83.0 m`; with the maximum registered
sensor-ring radius this still leaves more than the required 30 m separation.

These figures are only a design check.  The generated stochastic sensor
trajectories, exact FoV evaluation, target dynamics, physical graph, and
blockage overlap must pass the existing scenario validator before the scene is
used for source discovery or tracking.

## Generated-geometry result

Seeds 41, 43, and 47 pass the exact generator and scenario validator at both
scales.  All six runs have zero blackout, one nearest-formation handover per
target group, minimum sensor-target separation between 49.54 m and 52.96 m,
and an all-time connected formation chain with `F-1` edges and maximum degree
two.  The exact sensor-ring FoVs create more overlap than the centre-only
calculation, but the overlap remains scale-controlled: X36/M24 mean-visible-
formation ratios are `1.0335--1.0391`, and local sensor-load ratios are
`1.0058--1.0092`.

## Intended scale gate

For M24 and X36, require the ratio of mean visible formations to remain within
`0.90--1.10`, while blackout stays below 1% and the generated physical graph
remains a connected sparse chain.  The three-seed geometry gate passes.  Both
presets nevertheless remain development-only until source-side action
opportunity and paired tracking are evaluated under a separately frozen
protocol.
