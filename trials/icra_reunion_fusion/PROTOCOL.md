# Mobile-robot reunion fusion screen v1

Registered 2026-09-08 before viewing any tracking output from these inputs.
Parent checkout: `codex/icra`, `5f11b2e3`. The user asks for a promising ICRA
direction with a scene change and modest fusion changes, without a motion
planner, controller, learning stack, or large engineering rewrite.

## Question and decision scope

Do complementary local views and target births/departures during separation
create an identifiable **fusion** problem at robot encounters? Does a small
causal fusion change outperform simple existing fusion controls at identical
routes, observations, delivery draws and charged wire budgets?

This is a direction screen, not a new-method or publication-readiness claim.
Geometry grouping is frozen to a per-component bidirectional distance MST.
The previous geometric-grouping screen is retained unchanged. No additional
ICASSP best-table entry follows from this different scenario.

## Scenes and sensing

- Reuse the previous smooth split/rejoin and boundary-churn trajectories,
  scaling all lengths by 0.4: eight robots, 120 frames, dt=0.5 s, 24 m radio.
  Maximum speed remains below 3 m/s and acceleration below 2.1 m/s^2. These
  are prescribed kinematic patrol paths, not validated robot controllers.
- Range-limited circular sensing is 14 m (original pilot equivalent 35 m),
  pD=0.9, Cartesian measurement std=0.4 m, Poisson clutter mean 1 over a
  fixed rectangle [-52,52] x [-32,48] m. Global poses/clock are assumed known.
  No images, LiDAR, obstacle detector, localization error or CARLA execution.
- `split_latebirth`: two initial targets; two further targets are born while
  the teams are separated. All four survive the episode.
- `churn_departure`: same birth configuration, but the third target departs
  after frame 90. This deliberately tests stale positive beliefs as well as
  acquiring positive information. No truth departure notification is sent.
- `split_no_new`: only the two initial targets actually exist; the same
  later candidate priors are still supplied. This is a false-positive control.
- Six fixed known entrance regions are specified before truth: initial
  regions 1/2 and candidate regions 3--6 at frame 35. All robots receive the
  same broad priors r=0.1, position std=3.2 m, velocity std=0.2 m/s. Actual
  offsets and target count are not supplied. Candidate regions 5/6 are empty.
  Birth-region/time knowledge and globally consistent birth-label keys are
  simplifying assumptions. This does not solve independent track association
  or measurement-driven birth. Actual target offsets are sampled separately.
- Same LMB prediction, LBP update, eight-component GM cap, mixture-aware
  approximate KLA primitives and MAP extraction as the earlier pilot.
  Surviving local posteriors are retained across encounters. No message replay.

## Arms (all defined before tracking results)

1. `local`: no communication, context only.
2. `kla`: existing support-renormalized KLA.
3. `fov`: existing fov-aware absent-label censor with its existing thresholds.
   This is a repository baseline, not a full published multi-view algorithm.
4. `lineage`: `fov` plus the existing V284 untouched-prior exclusion. A local
   scheduled update has observation lineage if GM-mean-weighted pD>0; missed
   detection counts too. Only actually participating lineage is propagated.
5. `mil`: existing shared-label LMB-MIL with zero existence for absent inputs.
   This is the shared-label objective baseline, not a full unequal-FoV and
   label-matching reproduction.
6. `recent`: same absent-label censor as `fov`; for each represented label,
   multiply the scheduled-and-delivered Metropolis weights by
   `0.25 + 0.75 exp(-direct_opportunity_age_seconds/5)`, then normalize.
   Never directly observed sources receive 0.25. The timestamp is **local
   direct observation opportunity**, not a relayed freshness claim: the
   current predicted GM has weighted pD>0 before the scheduled local update.
   Both positive and missed observations count. Receiver own timestamp is
   local; neighbor timestamps are serialized in the received posterior.
   Missing labels retain their base weight so that the existing observable
   absence rule remains a legitimate negative input. No truth, target ID,
   future trajectory, realized packet outcome or offline error selects weights.

`recent` is a deliberately simple hypothesis, not claimed novel. It can
underweight useful relays and does not establish consistency after reunion.
The 0.25 floor and 5 s decay are frozen; no parameter grid follows failure.

## Fairness, packets and records

All communicating arms use exactly 2(N-C_t) directed attempts/frame, 0.1
independent directed loss, the same link uniforms, and one synchronous round.
Dropped weights return to self. Each actual posterior is serialized and
decoded into receiver input. Every arm carries identical metadata fields
(lineage flag and last direct opportunity) and a 16384-byte padded packet.
The larger cap accommodates six labels with up to eight GM components each.
Unpadded bytes, failed attempts, delivered bytes, and padding are separate.
The same 64 B/node uplink plus 64 B/node downlink geometry coordination is
charged; it is modeled reliable centralized control, not a distributed
agreement implementation. No bandwidth-efficiency claim follows from padding.

Save variable-cardinality truth, measurements, uniform draws, trajectories,
pre/post estimates, label existence, direct opportunity timestamps, weight
changes and all communication counts. Source hashes are frozen before full
runs; preserve both raw and LF-normalized hashes to avoid cross-platform
line-ending ambiguity.

## Run and metrics

- Seeds 2801/2802/2803 are paired development screens. Three trajectories
  are deterministic mechanism styles; seeds do not make independent motion
  scenes. `smoke` runs four frames only for API/serialization verification.
- Full-episode position OSPA p=2/c=12 m and count MAE are primary. Also report
  GOSPA p=2/c=12/alpha=2 with localization/missed/false **squared costs**,
  conditional assignment RMSE with support, worst-node mean and p90 OSPA.
- All extant targets are scored at all nodes, including inaccessible ones;
  truth visibility is diagnostic only. Report unobserved targets explicitly.
- For actual new targets: node-level time to the first three consecutive
  estimates geometrically assigned within 2 m, censored at episode end;
  summarize the other team's nodes separately. This is geometric acquisition,
  not proof of identity continuity. Common-target RMSE supports are audited.
- For target departure: after frame 90 report false-target GOSPA cost and
  count error. For reunion: mean OSPA for the next 10 frames, truncated at
  next disconnection. No already-satisfied 5 m recovery-time ranking is used.
- independently recompute assignment metrics from saved truth/estimate sets
  in Python; metric recomputation is not an independent method validation.

Continue a candidate only if relative to **both** `kla` and `fov` it improves
mean OSPA by at least 5% on both event scenes, has no scene count-MAE increase,
no more than 5% worst-node regression, and does not raise no-new-target false
cost by more than 5%. Investigate matched-support localization before judging
a count-recovery gain. If only a published/common alternative works, report
the scene's fusion sensitivity; do not rename that alternative as innovation.
If these controls solve the problem, or the small candidate fails, stop its
parameter search and use the diagnostics to narrow the next mechanism.

## Literature status at registration

Official-web tool, arXiv/Crossref HTTP and OpenAlex fallback were attempted
but failed with network/TLS errors on this session. Prior repository notes
already identify multi-view LMB, MIL, information weighting, active tracking
and network reconfiguration as existing work. These are pointers, not a fresh
novelty review. External verification remains open; no first/novel or ICRA
acceptance claim is permitted from this screen.
