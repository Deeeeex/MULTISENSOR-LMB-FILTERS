# Lossless omission candidates on fixed Gaussian-evidence histories

This diagnostic begins after the complete nine-sequence M-GE development
audit and while its unchanged 25-sequence transfer run is active. It does
not change either tracking implementation, select an accuracy variant or
reinterpret any native recorded byte count.

For every saved primary packet, the old 232 B/object base and 32 B header
already contain the local posterior, existence increment and both branch
gates. Consider one additional uint8 tag per object, followed by the original
15 double ratio parameters only for objects that need them. Compare exactly
three fixed omission rules, without tuning a numerical threshold:

1. Exact zero: omit only if all 15 encoded doubles are exactly zero.
2. Curvature: additionally omit ratios rejected by the existing primary's
   fixed per-source curvature criterion. Carry that admission decision in
   the tag; receiving code must honor it when forming the residual.
3. Current gate: additionally omit ratios with exactly zero active branch
   gate (g_positive if delta>=0, g_negative otherwise). Such a source has
   kappa=0 for every possible receiver beta. The tag also carries the source
   curvature decision so diagnostics can retain it if another source
   triggers evaluation.

No source can omit based on a receiver's future label matching, beta or
delivery draw. Do not omit merely because delta=0: nonzero spatial evidence
can coexist with zero existence increment. Do not apply approximate zero
thresholds, quantization, eigenvalue clipping or altered gates.

Read the actual saved 15 wire doubles, join local scalar and Gaussian records
by time/source/original label, and reconstruct curvature from the recorded
posterior covariance and transmitted precision difference. Require existing
input hashes and previous independent full Gaussian audit. Compute each
packet length as 32+233*n+120*retained, then use the original delivery draws,
16384 B fragmentation and 256 B/frame control convention.

These are projected lengths for explicitly specified codecs, not measured
native packets. A candidate codec would still require actual encoder/decoder
roundtrip, exact fused-state parity and recorded physical packet lengths
before any communication-saving claim. Existing 352 B records remain the
native results for the current M-GE round. All developer and transfer
trajectories already informed development; no independent-test claim.
