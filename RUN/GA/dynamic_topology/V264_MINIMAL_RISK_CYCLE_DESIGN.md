# V264 minimal risk cycle

## Evidence inherited from the tree screens

V261, V262 and V263 agree that formation-level routing changes the tracking
outcome, but they expose a tree constraint rather than a missing trigger.
V262's stable single-source route gives a large F4 localization gain while
hurting F4 set error and F2.  V263 makes every source-to-F4 distance physically
shortest and improves network E-OSPA and consistency, but loses the direct
F1-F2 relation and causes severe F2/F3 RMSE regressions.

At the entry state, V242 is the chain

```text
F1 -- F2 -- F3 -- F4
```

and V263 is

```text
F1 -- F3 -- F4
       |
       F2
```

Their union has four formation edges.  It improves F1-to-F4 and F1-to-F3
relative to V242, preserves F1-to-F2 and F2-to-F4, and does not increase any
formation-pair shortest-path distance relative to either tree.  No
three-edge spanning tree can preserve all four relations at once.

## Minimal relaxation

V264 first runs the unchanged posterior-only V263 selector.  On an active
risk-rooted page it considers only currently physical incumbent edges that
the rooted tree displaced.  An edge is accepted only if adding it:

1. does not increase any pairwise formation hop distance relative to either
   the current V242 tree or the V263 rooted tree;
2. strictly repairs at least one rooted-tree dilation; and
3. can reuse one physical gateway message in each direction from the current
   or immediately previous selected route.

Among admissible edges, V264 maximizes total repaired pairwise distance, then
prefers a previously used edge and immutable formation UID order.  In the
opened M24 state this uniquely restores F1-F2.  The two directed messages use
the existing V242 cross-residual weight 0.05; each receiver's self weight is
reduced by the same amount, preserving nonnegative row-stochastic KLA weights.
There is no new weight sweep.

Active pages use `N + 2F` messages, or 32 for M24.  Inactive pages return to
the `N + 2(F-1)` V242 count.  This is the smallest undirected-edge relaxation
of the tree architecture and adds a constant two messages independent of
network size.  The primary communication gate is therefore attempted bytes
below the corrected static baseline, not equality with V242's minimum count.

## Frozen continuation gate

Run one paired `t=57--73` continuation from the same V259/V242 state.  The
mechanism must activate exactly one extra formation edge, dominate both tree
distance vectors on every active page, remain physical and strongly
connected, keep row-stochastic weights, and match 32 messages when active and
30 otherwise.

Proceed to a full M24 arm only if F4 event E-OSPA and RMSE both improve,
network E-OSPA, RMSE and consistency stay within the inherited 2% guards, no
formation regresses by more than 3%, and spliced full-episode attempted bytes
remain below static.  Failure closes topology-only routing on this event and
moves the next decision to label-selective KLA trust before learning.

The policy uses current LMB summaries, current physical links, current link
reliability and past selected routes.  It reads no truth or future outcome.
The current-network control synopsis remains centralized and uncharged, so
this screen cannot establish deployment cost, validation or generalization.

## Result and decision

V264 activated the F1-F2 protective edge on every risk-rooted page from
`t=57` through `t=66`.  Each active graph used 32 messages, dominated both
tree distance vectors, remained physical and strongly connected, and reused
the frozen 0.05 residual weight.  Spliced attempted bytes remained 9.904%
below the corrected static baseline.

Relative to V242, network E-OSPA improved by 2.281%, consistency by 4.312%,
and F4 event RMSE by 25.689%.  Network RMSE nevertheless regressed by 5.230%,
F4 event E-OSPA regressed by 0.593%, and F2/F3 RMSE regressed by 32.070% and
46.336%.  Restoring F1-F2 reduced the network-RMSE loss from V263's 7.723% to
5.230%, so the protective relation matters, but one extra full-posterior edge
cannot remove the collateral label effects.  The registered gate fails and a
full M24 run is not authorized.

Together, V261--V264 show that topology changes have genuine leverage but a
formation edge acts on every transmitted label at once.  Replacing a tree
edge sacrifices another relation; adding the missing full-posterior edge
reduces that sacrifice but still perturbs unrelated labels.  The next method
decision is therefore label-selective routing: keep the V242 full-posterior
backbone unchanged and add only the at-risk Bernoulli component on a physical
shortcut.  This uses the label factorization of the LMB KLA instead of asking
one global formation graph to serve conflicting label-level needs.
