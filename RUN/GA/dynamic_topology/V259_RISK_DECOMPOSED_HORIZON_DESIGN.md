# V259 risk-decomposed residual routing

## Decision inherited from V258

V258 falsified the claim that the existing one-step, network-mean Pareto
surrogate can repair the M24 formation-4 localization tail.  In the fixed
posthoc window `t=58--73`, it admitted one local residual only at `t=59`.
The event-window RMSE then changed by `-2.122%` relative to V242 even though
the complete-episode RMSE changed by `+2.140%`.  One isolated action therefore
had delayed, sign-changing recursive effects; neither the posthoc window nor
the old scalar surrogate should be tuned further.

The useful structural result remains V242: one local directed cycle per
formation plus one bidirectional gateway for every formation-tree edge gives
a physically feasible, strongly connected `N + 2(F-1)` posterior-message
backbone.  V259 keeps that graph as the mandatory fallback.

## First-principles risk split

For a matched LMB label, the Bernoulli KLD separates into an existence part
and an existence-weighted spatial-density part.  The two terms imply different
communication actions:

1. **Localization-tail risk.**  A label remains supported, but its positional
   density becomes broad.  The candidate action is an omitted intra-formation
   residual KLA input, because an additional local view can reduce positional
   uncertainty without changing the formation tree.
2. **Label-support risk.**  A label is supported elsewhere in the network but
   weak or absent in one formation.  The candidate action is a sensor-level
   re-embedding of an existing cross-formation gateway, so a more informative
   sender or a more useful receiver carries the missing support without adding
   a posterior message.

This split is not a learned weighted feature score.  It changes the action
space according to the failure mode, after which a mixture-aware LMB-KLA
counterfactual ranks only the relevant candidates.  Any later learned model
may approximate finite-horizon value inside each action family; it may not
own connectivity, fusion weights or the byte cap.

## Scale-normalized observable statistics

At each pre-topology local-update page, V259 forms the union of label keys and
uses only local LMB posteriors, the registered formation membership and the
OSPA position cutoff `c`.

For an active label `ell` at sensor `i`, the conditional localization
contribution is

`u_i,ell = min(tr(P_i,ell^pos) / c^2, 1)`.

Only labels active at at least 80% of a formation are eligible for this mode.
Sensor values are aggregated by their lower quartile before an upper-tail mean
is taken across labels.  The lower quartile detects a formation-wide loss of
precision instead of reacting to one isolated sensor.  This avoids V246's
failure mode in which many moderate labels or a few weak sensors dominate one
systemic localization tail.  Existence is handled by the separate support
mode and by the active-label gate; it is not allowed to suppress the
conditional error of an already reported label.  The trace also retains the
formation mean for comparison.

For support, missing labels are assigned existence zero.  A label's network
reference existence is the maximum formation-median existence.  For every
network-supported label, the one-sided formation deficit is the bounded
Bernoulli existence divergence

`1 - exp(-D_Ber(r_net,ell || r_f,ell))`

when `r_f,ell < r_net,ell`, and zero otherwise.  The mean and upper-tail
deficits are bounded in `[0,1]` and do not depend on the number of sensors or
formations.  Because different FoVs legitimately produce many absent labels,
V259 also records each formation's prominence above the network-median deficit
and its relative coverage/count deficit.  Temporal persistence will be
evaluated from past pages rather than by reading future measurements.

## Evidence sequence

The first V259 run is deliberately passive:

1. rerun only the deterministic V242 arm on the already opened corrected M24
   seed 1301;
2. capture every pre-topology local posterior and compute both risk modes;
3. verify that passive capture reproduces the recorded V242 outcome metrics;
4. compare the observable traces with already opened truth-level errors only
   offline, to determine whether the modes are identifiable before coding a
   controller.

The trace is useful only if localization-tail risk identifies the F4 event
without being hidden by F2's larger mean risk, and support risk identifies the
late F1 label collapse.  Thresholds must be dimensionless, use past/current
pages only, and yield a sparse duty cycle.  If these conditions fail, V259
stops before another full tracking arm.

If they hold, the next experiment will use the frozen V242 backbone and two
separate bounded action queues.  A localization queue may add at most one
local residual; a support queue may re-embed at most one existing gateway.
The controller will use persistence/hysteresis and a finite-horizon value
estimate, with deterministic fallback whenever the relevant risk decrease,
physical projection or communication bound fails.  M24 must pass before the
same normalized policy is frozen for X36.

## Evidence boundary

The seed-1301 trace is development evidence and may use recorded tracking
outcomes only after the causal observables have been generated.  The online
risk statistics read no truth, future measurements or future delivery
outcomes.  A passive trace can establish feature/action alignment, not
tracking gain, generalization or a paper claim.
