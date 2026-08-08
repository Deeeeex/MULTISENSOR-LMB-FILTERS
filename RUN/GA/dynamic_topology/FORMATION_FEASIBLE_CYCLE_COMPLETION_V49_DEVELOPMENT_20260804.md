# V49 feasible-cycle completion development checkpoint

## Decision

The corrected seed-41 all-B4 structural replay passes its development gate:
271 of 320 windows select a cycle, all 271 selected windows improve the
posthoc four-page structural propagation factor, and no window is worse than
V46.  This authorizes a non-scoring real-filter runtime smoke.  It does not
yet authorize tracking, total-byte, distributed-commit, or no-worse claims.

The main finding is not that residual traffic should be spread across phases.
It is that V46's minimum-edit projection often leaves the formation layer as a
tree even when the current physical graph supports a cycle.  Completing that
cycle under the same directed message count produces material propagation
headroom on both M24 and X36 non-radial scenes.

## Frozen development configuration

- input page: current geometry, current physical adjacency, current link-loss
  probabilities, sensor/formation physical UIDs, and the exact V46 reference;
- proposal: rank physical Hamiltonian cycles by worst then mean directed link
  reliability;
- exact evaluations: at most the top three feasible proposals plus V46;
- pulse: fixed synchronized B4 phase 1 for every arm;
- weights: dominant 0.70, active residual 0.20;
- missing-neighbor mode: renormalize;
- switch gate: cycle must improve the exact frozen-page factor by at least 1%;
- fallback: exact input V46 adjacency and weights, with ties going to V46;
- one-step defer: disabled.

The initial frozen-page result hash is
`a3027dd7e47b0c8e7c11f65862598aac3715a833dd7f2e87a16e3b56af9cf902`.

## Seed-41 focus-page result

| Scale | Style | V46 sync \(\rho_4\) | Selected \(\rho_4\) | Route gain | Action |
|:--|:--|--:|--:|--:|:--|
| M24 | radial | 0.910924677 | 0.910924677 | 0.000% | V46 fallback |
| M24 | convoy | 0.983293918 | 0.907194317 | 7.739% | cycle |
| M24 | relay | 0.984695563 | 0.900936574 | 8.506% | cycle |
| M24 | crossing | 0.979765024 | 0.916717315 | 6.435% | cycle |
| X36 | radial | 0.971302481 | 0.971302481 | 0.000% | V46 fallback |
| X36 | convoy | 1.003969090 | 0.968306576 | 3.552% | cycle |
| X36 | relay | 1.006216930 | 0.969151080 | 3.684% | cycle |
| X36 | crossing | 1.001511950 | 0.967782631 | 3.368% | cycle |

The 1% gate selects a cycle in six of eight cases and returns V46 in both
radial cases.  All selected cases preserve the exact total route budget of
\(2N\) directed messages per full step and the synchronized-B4 budget of
\(5N\) posterior-message opportunities per four phases.

For the non-radial MST baseline, cycle completion changes message composition:

| Scale | Formations | V46 cross residual | V49 cross residual | Cross delta | Local delta |
|:--|--:|--:|--:|--:|--:|
| M24 | 4 | 6 | 8 | +2 | -2 |
| X36 | 6 | 10 | 12 | +2 | -2 |

Therefore equal message count must not be reported as equal byte cost until
posterior payloads and route/control traffic are measured.

## Seed-41 focus-window time-varying replay

The policy selects a route from the completed burst page only.  The posthoc
audit then uses the actual dominant route and actual link-loss probabilities
on each of the four executed pages.  The candidate cycle replaces only the
burst-page residual layer, exactly as synchronized B4 requires; future pages
are never exposed to the selector.

| Scale | Style | Snapshot gain | Posthoc four-page gain | Action |
|:--|:--|--:|--:|:--|
| M24 | radial | 0.000% | 0.000% | V46 fallback |
| M24 | convoy | 7.739% | 7.739% | cycle |
| M24 | relay | 8.506% | 8.506% | cycle |
| M24 | crossing | 6.435% | 6.435% | cycle |
| X36 | radial | 0.000% | 0.000% | V46 fallback |
| X36 | convoy | 3.552% | 3.552% | cycle |
| X36 | relay | 3.684% | 3.684% | cycle |
| X36 | crossing | 3.368% | 3.368% | cycle |

Six of eight windows select a cycle, all six improve the posthoc four-page
factor, and neither fallback window is worse than V46.  The result hash after
aligning focus starts to absolute B4 phase 1 and hardening the replay schema is
`0376dd3d9c04b6fb1e98231f375d3a6d12345878abbe76f20c2c0d384a7733ab`.
The canonical hash excludes measured wall-clock timing but retains proposal
counts, routes, factors, decisions, budgets, and every claim-boundary flag.
The snapshot and posthoc gains happen to match on these eight windows because
their relevant route/reliability state does not change enough to alter the
four-page score.  This equality is an observed result, not a method invariant.

Focus mode contains only one window per case, so route-pair and gateway churn
are undefined across windows and are recorded as zero by convention.  Churn
must be assessed in the all-B4 replay, not inferred from this table.

## Seed-41 all-B4 structural replay

The corrected replay starts at every absolute phase-1 boundary over all eight
160-step scenarios, giving 40 windows per scenario and 320 windows in total.
The policy still reads only the completed current page.  The next three pages
are used only by the posthoc structural audit.

| Scale | Style | Cycle windows | Mean over all windows | Selected median | Selected range | Pair-set changes | Cycle/fallback changes | Gateway symmetric difference |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|
| M24 | radial | 16/40 | 2.036% | 5.184% | 1.096%--5.963% | 3 | 2 | 52 |
| M24 | convoy | 40/40 | 8.256% | 7.739% | 7.739%--9.446% | 2 | 0 | 16 |
| M24 | relay | 40/40 | 8.750% | 8.506% | 8.506%--9.481% | 2 | 0 | 24 |
| M24 | crossing | 40/40 | 6.064% | 6.408% | 3.179%--8.679% | 4 | 0 | 128 |
| X36 | radial | 15/40 | 0.762% | 2.012% | 1.482%--2.462% | 7 | 6 | 142 |
| X36 | convoy | 40/40 | 3.846% | 3.552% | 3.552%--4.948% | 6 | 0 | 72 |
| X36 | relay | 40/40 | 3.808% | 3.684% | 3.580%--4.371% | 4 | 0 | 52 |
| X36 | crossing | 40/40 | 3.454% | 3.372% | 2.755%--4.609% | 16 | 0 | 300 |

All 240 non-radial windows select a cycle and improve the audited factor.
This includes every X36 convoy, relay, and crossing window; their worst
selected improvement is 2.755%.  The radial scenes exercise the 1% gate and
exact V46 fallback instead of forcing a cycle on every window.  Globally,
271/320 windows are selected and positive, the negative-window count is zero,
and the empirical no-worse fraction is 1.0.  The canonical result hash is
`b140a0285cb89e703c6d9abfa60042fc14b62a9c3bbd045a018bf6fc4589196b`.

The planning and posthoc factors are numerically identical in this seed-41
matrix.  Diagnostics show that the route and loss probabilities on the
active edges relevant to these B4 products remain unchanged within each
window.  This is a property of the registered scenarios, not a theorem or an
online guarantee.

X36 crossing is the main control-plane warning: the selected formation-pair
set changes 16 times over 39 transitions and the accumulated cross-gateway
symmetric difference is 300.  The structural improvement is stable, but a
deployable method still needs route-hash dissemination, atomic fallback, and
explicit control-byte accounting.

## Top-3 proposal versus exact-enumeration oracle

An independent focus-window comparison keeps the phase, weights, 1% gate, and
certificate fixed, changing only candidate generation.  The exact arm scores
every feasible physical cycle and is a development oracle, not a scalable
runtime policy.

| Scale | Style | Top-3 gain | Exact gain | Oracle gap | Top-3 / exact evaluations |
|:--|:--|--:|--:|--:|--:|
| M24 | radial | 0.000% | 0.000% | 0.000 pp | 4 / 4 |
| M24 | convoy | 7.739% | 7.739% | 0.000 pp | 4 / 4 |
| M24 | relay | 8.506% | 8.506% | 0.000 pp | 4 / 4 |
| M24 | crossing | 6.435% | 6.435% | 0.000 pp | 2 / 2 |
| X36 | radial | 0.000% | 1.105% | 1.105 pp | 4 / 61 |
| X36 | convoy | 3.552% | 4.919% | 1.366 pp | 4 / 61 |
| X36 | relay | 3.684% | 4.061% | 0.377 pp | 4 / 11 |
| X36 | crossing | 3.368% | 3.918% | 0.550 pp | 4 / 19 |

The top-3 proxy is exact on these M24 focus pages but leaves measurable
headroom on every X36 focus page.  Dense X36 exact enumeration takes about
15 seconds per page on this development machine, compared with about 0.8
seconds for top-3.  Therefore V49 may claim exact certification of evaluated
proposals, but not global or near-global optimality.  A GNN or combinatorial
model is best positioned as a causal top-k proposal generator whose outputs
remain subject to the same exact gate and V46 fallback.  The top-3 and oracle
artifact hashes are respectively
`0376dd3d9c04b6fb1e98231f375d3a6d12345878abbe76f20c2c0d384a7733ab`
and
`46600459a9be9fbf06eeae646adc4d8324d4e8d68192b4d4505a86e1bd7e9bca`.

## Full32 physical availability

The availability audit spans eight styles, seeds 41/57/73/89, and all 160
pages per case: 5,120 physical pages in total.  Every page contains at least
one physical Hamiltonian formation cycle, so the cycle action is available on
100% of these pages.  The artifact hash is
`691f36270b201716d259f0e0992704ab700a6811571332b92620a839f87e1041`.

The UID-first cycle edge set is constant in radial/convoy/relay M24 except for
two changes in each M24 crossing trajectory.  X36 radial changes three times,
X36 convoy/relay remains constant, and X36 crossing changes six or seven times
over 159 transitions.  These are heuristic cycle changes, not yet the churn of
the exact top-3 selected sensor route.

## Attribution of the rejected defer arm

In the earlier three-arm exploratory screen, route completion contributed
roughly 3%–8% while one-step defer contributed only 0%–0.22%.  The defer action
is below the existing 1% materiality threshold and is removed from the primary
method.  It may remain as an explicitly labeled ablation, but no route gain may
be placed in a defer field or claim.

## What this checkpoint does not prove

- The route gate still uses a frozen current-page score.  The time-varying
  score is a posthoc audit; one seed and 40 correlated windows per scenario
  do not prove a population-level or online no-worse guarantee.
- The runner materializes planned sensor trajectories for development, even
  though each page-level policy receives only its current slice.
- The exact propagation factor is not a tracking metric and does not bound
  Bayes updates, label support changes, mixture approximation, or pruning.
- The top-3 proposal and V46 fallback are not atomically disseminated to the
  network.  Split execution, loss, retries, latency, and route discovery bytes
  are not modeled.
- Equal message count is not an equal-total-byte result.
- No posterior, truth, tracking score, or realized delivery draw was used in
  this checkpoint.

## Next authorization gate

The all-window structural gate authorizes only a short, non-scoring real-filter
smoke.  It must compare V49 with V46 synchronized B4 under the same
physical-UID-keyed delivery-uniform tensor and the same directed posterior
message counts `[2N,N,N,N]`.  It must independently replay the V49 policy,
verify installed route/weight hashes, confirm posterior non-interference, and
avoid reading tracking outputs.

After that smoke passes, the next research gates are: correlated-loss
sensitivity; route-hash dissemination and atomic fallback; control and payload
byte measurement; and multi-seed episode-level tracking evaluation.  The
top-3 proposal rule remains a heuristic until it is replaced or its oracle gap
is bounded.
