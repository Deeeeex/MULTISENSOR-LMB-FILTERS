# Bit-preserving zero-vector Gaussian codec

This is a codec verification, not a new fusion candidate. The underlying M-GE
and all three ablations were fixed before the preceding development run.
Their original 352 B packet results and ongoing 25-sequence runs are retained.

The preceding complete development packet diagnosis found many exactly zero
15-double Gaussian ratios. Its one-byte tag projection did not preserve the
bit patterns of signed zeros. This concrete codec instead uses one uint16
tag per object: the high bit means that the original 120 B vector follows;
otherwise the low 15 bits preserve each coefficient's zero sign. The old
32 B header and 232 B/object base bytes pass through unchanged. Packet length
is exactly 32+234*n+120*k, where k counts vectors with any coefficient !=0.
No numerical tolerance, quantization, curvature rejection omission or gate
change is allowed. Every input double's 64 bits must roundtrip unchanged.

The receiver reconstructs omitted zero sign bits and copies nonzero vectors
exactly, so all original Gaussian natural parameters remain identical. No
fusion source, gate, record, extraction or observation should change. The
original 352 B codec is the component control for this transport-only check.

Before tracking, verify empty, all-zero, mixed signed-zero and nonzero vectors,
opaque base-byte identity, actual Bernoulli packets, and the original Gaussian
fusion fixtures. Freeze source after units and before replay preflight.

Preflight full development 0000 under both link conditions, using the original
four Gaussian methods with this codec. Require complete trajectory, all
original local and fusion records and metric parity to the preceding
immutable outputs, except explicitly changed byte counts and runtime. Verify
actual packet tags, restored 64-bit coefficient equality and physical lengths.

After parity, register one complete codec replay of every primary development
sequence and every primary 25-sequence transfer trajectory after its original
run succeeds. Other ablation codec lengths may be checked on saved ratio
vectors with opaque fixed-width base bytes, but must be identified as saved
payload codec results rather than new native tracking. The primary's actual
full replays remain the completion criterion for an end-to-end communication
claim. Do not select an accuracy variant based on any codec result.

This preserves the original per-message delivery trace and fragmentation
model. It does not establish latency or packet-loss effects of a real network.
All original real trajectories have already informed method development.
