# V97 positive-net addressable payload control

## Decision problem

V96 protects the smallest subset covering 80% of safely addressable rescue
mass.  It passes both M24 anchors but reaches only 1.566% and 4.955% gain on
X36.  Earlier duration attribution shows that early recovery weakens tracking,
whereas increasing persistent spatial coverage raises X36 gain monotonically.
The remaining question is therefore whether the 80% minimum cover discards
safe positive-value formations as the number of formations grows.

For formation `f`, let `H_f` be the current observable existence mass rescued
by withholding its complete cross-formation posterior and `B_f` the currently
cross-supported mass that may be lost.  V97 selects exactly

`A = {f : action available, no supported downward crossing, H_f > B_f}`.

Under the additive current-state surrogate, every selected formation has
strictly positive marginal net benefit and every excluded addressable
formation has nonpositive marginal benefit.  The rule therefore needs neither
an 80% coverage target nor a fixed top-k, and its selected population can grow
with network size.

## Matched experiment

Each opened anchor compares three arms frozen before tracking truth is scored:

1. static carrier route with full posterior payload;
2. the V96 minimum 80%-coverage set held for all three steps;
3. the V97 positive-net set held for all three steps.

All arms share the cached posterior, measurements, link uniforms, filter RNG,
static carrier graph, fusion weights, horizon and communication constraints.
V97 must improve the static baseline by at least 5% at all two M24 and two X36
anchors, with no regression in worst-sensor, weakest-formation, consensus,
attempted-byte or rolling-connectivity metrics.  V96 is a mechanism ablation;
the static full-payload route remains the primary baseline.

These anchors are opened development evidence.  Passing them does not by
itself authorize a validation or full-episode claim.

## Development result

V97 improves the static baseline by 8.225%, 8.066%, 2.412% and 7.174% at M24
t=104, M24 t=124, X36 t=72 and X36 t=100 respectively.  It improves every
anchor over V96 and passes all registered tail, consensus, communication and
rolling-connectivity constraints, but the strict cross-scale gate passes at
only 3 of 4 anchors.  X36 t=72 remains the sole failure.  The next method
question is whether the safe positive-net set changes during the three-step
window; the result does not justify further tuning of the initial coverage
rule.
