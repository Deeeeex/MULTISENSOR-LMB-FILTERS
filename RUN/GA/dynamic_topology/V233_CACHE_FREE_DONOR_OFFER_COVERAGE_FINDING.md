# V233 cache-free donor-offer coverage

## Design correction

V230 and V231 assumed that a remote source could rank receiver-relative
surprise from a recently received beneficiary posterior.  The recursive
filter can persist such a cache, but the communication history does not
guarantee that the relevant reverse-direction message exists.  A proposal
mechanism whose recall depends on that cache is therefore not deployable in
the general dynamic graph.

V233 removes this dependency.  The withheld action concerns one donor
formation, so only common physical sources inside that formation need to
participate.  Each participating source enumerates its active labels in a
deterministic two-phase schedule with at most 12 compact records per phase.
The label identifier assigns a coverage slot only; it is not a learned value
feature.  After both phases, the beneficiary coordinator jointly ranks the
visible remote summaries against its local need and may request at most one
complete Bernoulli-GM label through the existing safe KLA projection.

For `S` participating donor sources, `H=2` phases, and `M=12` records per
source and phase, the exact control charge is

`S x 16 B + H x S x (16 B + M x 24 B)`.

If each participating source has at most `H x M = 24` active labels, every
active label is offered exactly once.  The payload request occurs after the
coverage schedule, so early stopping cannot invalidate this guarantee.  If
the exact control plus payload charge does not fit the recycled communication
credit, the repair abstains.

## Same-state result

The opened X36 `seed=1301, t=133` screen contains two useful teacher rows.
For beneficiary F5, only S2 is a common reachable donor source; its 16 active
labels require 624 B of control, and the 1,456 B payload leaves 8,320 B of
certified net saving.  For beneficiary F6, S1/S3/S5 participate with
24/24/23 active labels; their control charge is 1,872 B, leaving 7,072 B after
the same payload charge.  Both teacher labels appear in phase two, and every
active label at every participating source is covered.

This is a proposal-coverage and byte-feasibility result, not a tracking
result.  The next decisive step is a beneficiary-side coordinator that uses
only the offered summaries and local posterior state, followed by a short
H=3 tracking comparison against the corrected V227 references.
