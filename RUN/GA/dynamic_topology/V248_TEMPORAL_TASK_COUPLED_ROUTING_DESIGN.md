# V248 corrected-scene routing comparison

## Question

V247 established a real time-aligned information dependency across every
initial-tree cut that fails in M24, X36 and X48.  V248 asks the first tracking
question on that corrected scene: relative to a fixed formation tree with
physical dropouts, does causal route repair improve estimation, and how much of
that benefit survives in the minimum communication backbone?

## Frozen M24 arms

All arms use the same V247 M24 scene, seed 1301, measurements, complete
mixture-aware LMB fusion implementation and 160-step evaluation window.

1. **Fixed formation tree:** keep the initial formation pairs; when a pair is
   physically unavailable, omit that input and return its KLA mass to self.
2. **Full causal repair:** preserve the previous formation tree while feasible
   and replace only the necessary edges after physical failure; retain two
   posterior inputs per receiver.
3. **Minimum causal backbone:** use one local directed cycle per formation and
   one bidirectional gateway pair for every formation-tree edge, for exactly
   `N + 2(F-1)` messages per step.

The V247 temporal visibility gate is rerun before any tracking arm.  It reads no
tracking outcome.

## Decision rule

The main comparison reports full-horizon E-OSPA, position RMSE, focus-window
inter-formation consistency, attempted posterior bytes and the weakest
formation E-OSPA/RMSE.  V248 remains single-seed development evidence.  If full
causal repair does not improve the corrected fixed tree, topology repair alone
is not the right story.  If it improves but the minimum backbone loses RMSE,
the next arm must restore only posterior-valued residual inputs.  Only a method
that improves all four network means and retains positive byte saving can
refresh the current-best table.
