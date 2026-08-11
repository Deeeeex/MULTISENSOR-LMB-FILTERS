# V105: H=8 protection-only causal attribution

## Question

V104 restores almost every harmful same-page handoff row to reference but
reproduces the V103 F1 and F6 regressions.  The remaining confound is that both
arms extend V101's control-only formation protection through t=79.  V105 asks
whether prolonged protection, without any topology handoff, creates the same
network mean gain and local harm.

## Exact arm

V105 uses the V103 protection sets:

`[1,2,4,5] -> [1,2,3,4,5] -> [1,2,3,4,5] -> all -> all -> all -> all -> all`.

At every page, adjacency and fusion weights are exactly the static fixed-
counter-clockwise route.  Only the formation-conditioned control-only payload
plan changes.  There is no gateway broadcast, receiver row replacement or
additional message.  The exact H=8 static outcome is reused after matching
the cache SHA-256; only the candidate is run.

## Decision

The strict V103 gate remains registered, but the causal comparison is more
important than the binary result.

- If V105 reproduces V103/V104, prolonged protection is the dominant cause;
  redesign activation age and safe deactivation before transport work.
- If V105 removes local regressions but loses the 5% mean, handoff supplies
  headroom and collateral harm; then receiver-label value control is needed.
- If V105 retains at least 5% mean and makes every formation positive, handoff
  is unnecessary and the deployable method should be protection-only.

No threshold or receiver set is changed after V105 outcomes open.

## Opened result

V105 lowers mean E-OSPA from 84.037151 to 79.617863, a 5.259% gain,
while saving 6.117% attempted bytes.  The gain is positive at every return
time and reaches at least 5.188% from t=76 onward.  However, formation gains
are `[-0.931, 4.805, 7.711, 8.970, 11.250, -0.021]%`, and the F6
non-gateway peers regress by 2.940% at the terminal time.  The strict gate
therefore fails.

This result reproduces V103/V104 without any topology handoff: the H=8
control-only protection schedule is sufficient to explain both the mean gain
and the local regressions.  The next method decision is a risk-aware
activation/deactivation rule for protection, not receiver-label transport or
GNN training.
