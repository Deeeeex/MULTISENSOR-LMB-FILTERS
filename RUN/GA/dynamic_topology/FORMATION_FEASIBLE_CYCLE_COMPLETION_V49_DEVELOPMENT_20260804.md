# V49 feasible-cycle completion development checkpoint

## Decision

The seed-41 focus-window time-varying structural replay passes.  Advance the
route-only feasible-cycle completion method to an all-B4-window structural
replay.  Do not advance one-step defer, tracking evaluation, or communication-
saving claims yet.

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

The development result hash is
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

| Scale | Style | Snapshot gain | Actual four-page gain | Action |
|:--|:--|--:|--:|:--|
| M24 | radial | 0.000% | 0.000% | V46 fallback |
| M24 | convoy | 7.739% | 7.739% | cycle |
| M24 | relay | 8.506% | 8.506% | cycle |
| M24 | crossing | 6.435% | 6.435% | cycle |
| X36 | radial | 0.000% | 0.000% | V46 fallback |
| X36 | convoy | 3.552% | 3.552% | cycle |
| X36 | relay | 3.684% | 3.684% | cycle |
| X36 | crossing | 3.368% | 3.368% | cycle |

Six of eight windows select a cycle, all six improve the actual four-page
factor, and neither fallback window is worse than V46.  The result hash after
fixing first-window churn accounting is
`4c725453448ddc4e3313a62900c4f5c4b8c97f703e50c618609c32a9cc196962`.
The snapshot and actual gains happen to match on these eight windows because
their relevant route/reliability state does not change enough to alter the
four-page score.  This equality is an observed result, not a method invariant.

Focus mode contains only one window per case, so route-pair and gateway churn
are undefined across windows and are recorded as zero by convention.  Churn
must be assessed in the all-B4 replay, not inferred from this table.

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
  score is a posthoc audit and the eight focus windows do not prove a
  whole-episode no-worse guarantee.
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

The next artifact may use only graph/link pages and must:

1. replay every B4 boundary in all eight seed-41 styles;
2. select from each completed current page without future-page access;
3. bind every proposed formation cycle and sensor assignment to physical UIDs;
4. audit the actual four-page factor separately from the planning snapshot;
5. report negative-window count, route and gateway churn, cycle run lengths,
   compute time, proposal work, fallback reasons, and message composition; and
6. keep tracking, atomic-execution, and total-byte claims closed.

Only a stable M24/X36 time-varying structural result can authorize a short
real-filter smoke.
