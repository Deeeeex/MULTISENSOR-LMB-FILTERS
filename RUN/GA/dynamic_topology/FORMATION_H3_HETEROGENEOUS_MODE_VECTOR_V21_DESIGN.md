# Formation H=3 heterogeneous mode-vector probe v21

## Question

v20 rules out a shared trust value as the missing repair mechanism.  The
best triple, formations 2+3+4 at mode 2, is close to safe but the
all-formation version is worse.  v21 asks whether different formations need
different third-step modes to close the remaining consensus debt while
retaining the strong tracking and communication gains.

## Frozen action space

Each action is a four-entry mode vector, one entry per formation:

- mode 1: registered reference row;
- modes 2, 3, and 4: the three current-state dynamic rows associated with
  trust 0.30, 0.50, and 0.70;
- the full teacher bank contains `4^4 = 256` vectors, enumerated
  lexicographically with the last formation varying fastest;
- this exponential bank is used only as an offline teacher instrument.  It
  is not the proposed deployment algorithm.

The v20 best-consensus vector `[1,2,2,2]` is the outcome-inspected center.
Before execution, v21 freezes every vector within Hamming distance two of
that center: one center, 12 distance-one vectors, and 54 distance-two
vectors, for 67 candidates in total.  This tests every way to alter any two
formation modes without silently tuning individual combinations.

## Frozen sequence and gates

- opened state: `m24-formation-fov / seed 211 / t=72`;
- first two mode vectors: `[1,1,3,1]` then `[1,1,1,4]`, reproducing the v18
  prefix `[9,13]`;
- third step: one of the 67 frozen heterogeneous candidates;
- controls: all-reference and prefix-plus-reference;
- the prefix and center outcomes must reproduce v20 within `5e-6` percentage
  points before candidate results are interpreted;
- strict feasibility requires all six targets to be nonnegative;
- a strong headroom result additionally requires at least `3%` mean tracking
  gain;
- physical, payload, exact-execution, rolling-B3, truth, repair, emergency,
  and infeasibility gates remain unchanged.

An outcome-free implementation preflight constructs the actual seed-211/t=72
bank and confirms that all 67 terminal candidates remain inside the t=74
physical graph and reference payload cap.  The current single-round posterior
proxy allows only one of them; it is recorded as a feature but is not used to
prune this oracle probe because earlier opened evidence shows that it misses
realized positive temporal actions.

## Decision boundary

A strict strong vector would establish a concrete teacher target for a GNN
or other structured value model, followed by a hard safety projection.  A
failure would only exclude this radius-two terminal neighborhood; it would
not prove that all 256 terminal vectors or all three-step vector sequences
are infeasible.

This is a privileged single-state mechanism probe.  Seeds 223/227, X36, and
final seeds remain unopened.

## Result

Both frozen reproduction controls pass, and all 67 candidates execute without
truth use, repair, payload emergency, infeasibility, or B3 failure.  None is
strict-feasible.  The strict oracle remains all-reference with `0%` gain.

The closest and best-consensus vector is `[1,4,4,2]`, with targets
`[+5.775, 0, +0.042, -1.527, +1.027, +1.074]%`.  It repays about 86.7% of the
original consensus debt while retaining strong tracking and positive
communication savings; consensus is its only failing target.  The current
single-round posterior proxy rejects this vector.

Thus heterogeneous terminal modes are useful, but optimizing only the third
step cannot undo the debt accumulated by the fixed first two actions.  The
next teacher must choose mode vectors earlier in the horizon and carry an
explicit predicted consensus-debt budget from the first action onward.
