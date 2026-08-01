# Formation H=3 complementary-sequence beam probe v18

## Mechanism hypothesis

The v17 mean-only ceiling shows a real but unsafe M24 opportunity at seed
211/time 72: changing formation 3 to trust 0.50 improves network-mean tracking
by `+5.988%`, while consensus falls by `-11.486%`.  Returning to the fixed
reference for two steps does not repair that imbalance.  v18 tests whether
different nonreference actions later in the same H=3 window can preserve part
of the tracking gain while restoring all five auxiliary targets.

This is not a learned or causal method evaluation.  It is an outcome-inspected
existence probe that decides whether complementary temporal composition is
worth modelling.

## Frozen local sequence bank

- preset / opened state: `m24-formation-fov / seed 211 / t=72`;
- first action: local-bank index 9,
  `formation-3-dynamic-trust-0.50`;
- possible repair actions: reference plus all trust modes for formations 1,
  2, and 4, indices `[1,2,3,4,5,6,7,11,12,13]`;
- each nonreference graph and fusion matrix is constructed once from the
  current posterior and then frozen; no future posterior rebuilds an action;
- a reference index invokes the registered fixed reference at that step;
- terminal targets: the original six H=3 gains, all required nonnegative;
- per-step gates: physical reachability, exact graph replay, payload not above
  reference, no truth use, no repair/emergency/infeasibility, and rolling-B3.

Pairs are deliberately excluded from this first probe.  A failure therefore
falsifies only local complementary sequences, not all temporal policies.

## Two-stage beam protocol

Stage one evaluates the all-reference arm and every sequence `[9,r,1]`, where
`r` ranges over the ten repair actions.  It must first reproduce the registered
`[9,1,1]` targets at the v17 ceiling.

Four distinct second actions are then retained by predeclared criteria, in
order: maximum network-mean gain, maximum consensus gain, maximum
minimum-formation gain, and maximum worst auxiliary margin.  Ties use the
smaller action index.  If criteria collide, remaining slots use a fixed
mean-minus-negative-auxiliary-debt score.

Stage two evaluates `[9,b,r]` for each retained second action `b` and every
nonreference repair action `r`.  Completed stage-one arms are reused rather
than rerun in the combined oracle.  This is a diverse beam, not exhaustive
enumeration.

## Decision

- A terminally strict-feasible sequence with mean gain at least `3%` is a key
  mechanism finding.  It authorizes broader opened-state sequence headroom
  tests and then a causal sequence-value model with exact safety projection.
- Weak positive gain below `3%` does not meet the M24 target.  The next probe
  may add pair/coordinated repair actions using the same opened state.
- No feasible improvement shows that local temporal composition is
  insufficient; it does not falsify all sequences because the beam and repair
  bank are restricted.

Seeds 223/227, X36, and all final seeds remain unopened.

## Result

The probe evaluates 47 unique sequences.  Only the all-reference sequence is
strict-feasible, so the strict oracle remains `[1,1,1]` with `0%` gain.  The
four selected second actions are `[13,12,1,11]`, exactly corresponding to the
predeclared diverse criteria.

The strongest near-feasible local sequence is `[9,13,12]`: formation 3 at
trust 0.50, then formation 4 at trust 0.70, then formation 4 at trust 0.50.
Its six targets are
`[+8.623, 0, -0.001, -3.717, +0.023, +0.024]%`.  Relative to `[9,1,1]`, it
repays about 68% of the consensus debt, raises tracking gain by 2.635 points,
and restores both communication targets, but it cannot close the remaining
consensus gap within H=3.

This is a useful negative result.  Temporal complementarity is measurable,
but one-formation-at-a-time repair is insufficient.  The next bounded probe
will keep the best opened prefix `[9,13]` and replace the third local action
with every conservative two-formation repair action.  Failure of that probe
will motivate a genuinely coordinated projection rather than further trust
or duration tuning.
