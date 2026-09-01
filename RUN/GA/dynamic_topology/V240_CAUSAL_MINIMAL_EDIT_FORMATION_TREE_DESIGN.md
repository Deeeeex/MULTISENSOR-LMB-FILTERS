# V240 causal minimal-edit formation-tree routing

## Why V227 is not a full dynamic-topology method

V227 rebuilds sensor-level gateway assignments from current geometry, but it
restricts every cross-formation message to the formation pairs registered at
the start of the episode. It can therefore change gateways inside a fixed
formation backbone, but it cannot replace a formation pair when that pair is
no longer physically reachable. The formation-braid probe stays globally
connected while deliberately changing which formation pairs are neighbours;
V227 then fails with `NoCrossAssignment`.

## Scene mechanism

The new `*-formation-fov-formation-braid` presets repeat one local module as
network size grows. Each adjacent formation pair completes a staggered
overtake in separated passing lanes. This changes the physical formation
order and replaces the bridges between neighbouring pairs without
disconnecting the network. M24, X36 and X48 contain two, three and four such
modules. All three use the same 120-degree, 300 m sensing envelope, a 270 m
communication radius and target service lanes offset by 170 m.

## Routing rule

At each step, V240 first reconstructs the previous formation tree from the
causal policy history.

1. If every previous tree edge remains physically available and admits
   distinct incoming gateway receivers, preserve the tree.
2. Otherwise enumerate the currently feasible formation trees and select
   lexicographically by:
   - maximum number of retained incumbent edges;
   - maximum bottleneck link reliability;
   - maximum total log reliability;
   - minimum physical distance;
   - immutable formation UID tie-break.
3. Within the selected formation tree, rebuild the physical-UID-stable local
   cycles and cross-formation gateway assignment.

Every receiver retains exactly two inputs and the V227
self/dominant/residual KLA weights. The policy reads current geometry, current
link reliability, immutable identities and previous policy history; it does
not read truth, future outcomes or posterior payload values.

## Structural result on seed 41

| Scene | First frozen-tree failure | Necessary reselections | Messages/round | All-time strong and physical |
|:--|--:|:--|--:|:--:|
| M24 | 71 | 71, 149 | 48 | yes |
| X36 | 55 | 55, 95, 108, 153 | 72 | yes |
| X48 | 45 | 45, 71, 102, 103, 135, 149 | 96 | yes |

The three scenes remain physically connected throughout. Their minimum
sensor-target separation is about 43 m, and focus-window target blackout is
4.3--6.7%. These are structural development results only; tracking gain and
generalization remain untested.
