# Independent voxel-index checker repair

The second input-preparation attempt exposed an exact-check mismatch in
four raw-input shards. A retained original PCD and its compressed public
archive range reproduce the first observed mismatch. For example, a
projected x coordinate of approximately -5.3e-18 m lies just left of zero.
The producer's explicit histogram correctly retains that side, while the
old independent expression `floor(2*(x+50))` rounds `x+50` to exactly 50.

The repaired independent checker uses `floor(2*x)+100` (and the analogous
y expression), applying the integer origin after floor. No tolerance is
relaxed, and the producer's grid, registration, native filter, input roster
and statistical gates remain byte-identical to the original freeze.
`GRID_VERIFIER_REPAIR.json` checks the actual raw mismatch and exact, left
and right neighbors of every x/y grid boundary. The old files and both
failed execution logs remain available. The successful input receipt names
the effective checker and its separate repair record.
