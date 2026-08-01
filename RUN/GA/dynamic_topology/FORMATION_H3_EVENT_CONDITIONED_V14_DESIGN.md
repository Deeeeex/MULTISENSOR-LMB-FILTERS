# Formation H=3 event-conditioned timing falsifier v14

## Question

The v13 M24 action bank fails its six-state strict-oracle gate at fixed times
60 and 72.  This v14 experiment asks one narrower question before changing
the action space: did the fixed times miss the intermittent states in which a
topology intervention is useful?

The candidate actions, H=3 return, six targets, strict non-regression rule,
reference fallback, M24 seeds, and final-seed embargo remain unchanged.  Only
the state-time selection rule changes.

## Truth-free event score

For each candidate time, the fixed-reference trajectory exposes only the
current local LMB posteriors, current link-drop probabilities, and the previous
selected topology.  The event score is

\[
s_t = \max(\bar d_{\mathrm{cross,robust}}-
                  \bar d_{\mathrm{within}},0)
      (1+\bar p_{\mathrm{drop,route,robust}}).
\]

The disagreement term uses label-wise Bernoulli existence and
covariance-normalized spatial moments.  It is high when sensors agree inside
each formation but formations hold meaningfully different posteriors.  The
link term upweights the same discrepancy when the currently selected
cross-formation route is under stress.  Both robust terms are an equal-weight
mean/tail average with a 34% upper tail.

The score does not use truth, E-OSPA, future measurements, future link
uniforms, seed identifiers as features, sensor identifiers, or formation
identifiers.  It is invariant to sensor permutation and formation relabeling.

## Frozen timing protocol

- preset: `m24-formation-fov` only;
- sentinel-training seeds: `211`, `223`;
- sentinel-development seed: `227`;
- candidate grid: `40:4:136`;
- selected states: top two score maxima per seed;
- non-maximum suppression: selected times must be at least 16 steps apart;
- deterministic tie break: earlier time first;
- return: candidate at the first step, fixed reference for the next two;
- action bank: reference + 12 singleton + six trust-0.30 pair actions;
- final seeds `251`, `257`, `263`, `269`, `271`: unopened.

The broad middle-episode grid is fixed independently of the three configured
blockage windows.  Event times may therefore fall before, during, or after a
blockage; their selection depends on the observable filter state rather than
the scenario's future schedule.

## Unchanged M24 gate

Each action must be nonnegative in all six offline targets: network-mean
tracking, minimum-formation tracking, worst-sensor tracking, consensus,
attempted bytes, and delivered bytes.  Across the six selected states, M24
must retain at least two positive states, at least one state with gain of 3%
or more, and mean strict-oracle gain of at least 2%.

- If the gate passes, timing was a material v13 defect.  The learned problem
  becomes joint event/action value prediction under exact fallback safety.
- If the gate fails, changing the predictor is not authorized.  The next
  change must broaden the topology action space while explicitly controlling
  formation and consensus risk.
- X36 teacher generation remains blocked until the M24 gate passes.

Passing this opened sentinel would authorize only further method development,
not a tracking, deployment, or cross-scale claim.
