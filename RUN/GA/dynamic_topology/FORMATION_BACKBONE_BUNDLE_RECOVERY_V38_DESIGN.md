# V38 registered-backbone input-bundle recovery

## Why a new method version is required

The positive v35/v36 result is a three-step, seed-211 development result on
`m24-formation-fov`.  Its reference route is a counter-clockwise formation
cycle.  The v5 convoy and relay scenes instead register an
initial-geometry spanning tree and place their link blockages on that tree;
the radial scene instead registers a four-formation ring.  Running the old
controller unchanged would therefore risk suppressing a different set of
formation links from those that define each scene mechanism.

V38 keeps the useful causal idea from v35--temporarily suppressing a
low-weight cross-formation input when doing so protects the current label
set--but changes the controlled object.  It acts on input bundles induced by
the scene's registered formation backbone.  The old v35 implementation and
evidence remain unchanged and retain only their original local scope.

## Shared reference route

The same construction is used for radial, convoy, and relay scenes.

1. Collapse the registered static sensor graph into an undirected formation
   backbone graph.  Only the registered formation pairs are retained; the
   particular sensor endpoints may be any currently physical pair between
   those formations.
2. Replace every backbone edge by one directed arc in each direction and
   compute a deterministic Euler circuit.  This uses only the static
   formation graph, not posterior, truth, future motion, or future link
   outcomes.
3. Split each formation's sensors across its Euler occurrences and concatenate
   the blocks into one sensor-level Hamiltonian residual tour.
4. Give every receiver a 0.70 high-weight local input and a distinct 0.05
   residual-tour input; the remaining 0.25 stays on self.

The residual tour contains exactly `2E` cross-formation messages for `E`
registered undirected formation pairs, one in each direction for every pair.
Thus the radial ring keeps all four of its links, while the convoy and relay
trees keep all three.  The tour is one directed cycle, so the reference route
is strongly connected at the sensor and formation levels.  With six sensors
per formation, the construction is admissible only when every formation-
backbone degree is at most six.  Nonphysical or degree-infeasible routes fail
closed during source-only preflight; no unregistered formation-pair fallback
is claimed.

## Causal input-bundle action

One action suppresses every low-weight residual input entering a selected
formation.  The high-weight local route is unchanged, and each removed 0.05
weight returns to that receiver's self weight.  A formation of backbone degree
`d` therefore saves `d` attempted messages in that step.  This is deliberately
an input-bundle action rather than an arbitrary edge search.

For every formation, the controller compares the reference route with the
single-bundle suppression under the current LMB posterior and the current
link-delivery probabilities.  The normalized expected-cardinality change is
called the **reference-relative cardinality protection score**.  A positive
score means that suppressing the input bundle avoids current existence-mass
dilution relative to the registered reference.  It is not a general measure
of information, and it does not claim that the reference or candidate is
closer to truth.

The unchanged numerical policy is shared across scenes:

| Item | Value |
|:--|--:|
| Dominant / residual fusion weight | 0.70 / 0.05 |
| Protection on / off threshold | 2% / 1% |
| Maximum existence-retention risk | 1% |
| Minimum supported-label retention | 80% |
| Decision-threshold crossings | 0 |
| Minimum retained protection coverage | 80% |
| Minimum disagreement improvement for staged release | 0.25% |
| Rolling connectivity reserve | B3 |

The base controller requests every safe bundle whose protection score passes
the hysteretic threshold.  If their joint suppression is unsafe, the lowest
score is removed until the exact label-retention and rolling-B3 gates pass.
For bundles already absent for at least one step, the recovery controller
tries nested releases from lowest to highest current protection score.  A
release is accepted only if the retained bundles preserve at least 80% of
the positive protection score and improve one-round expected posterior
disagreement by at least 0.25% relative to keeping the incumbent action.

The controller evaluates at most `3F+1` routes per step.  Unlike the old H=3
experiment bank, it does not enumerate `2^F` formation subsets.

## Evidence plan

The next stage is development evidence, not held-out validation.

- Scenes: radial `m24-formation-fov`, v5 convoy, and v5 relay.
- Development seeds: `[41, 43, 47, 53, 59]`.
- Windows: each scene's complete registered focus window.
- Candidate and reference use the same backbone-tour construction, posterior
  continuation, measurements, link uniforms, filter RNG, and scoring code.
- The only arm difference is whether the causal input-bundle controller may
  suppress and recover cross-formation bundles.

Before any tracking score is opened, a source-only run must cover all fifteen
scene--seed trajectories and freeze the complete runtime fingerprints.  It
must show no truth access, future-input access, nonphysical action, emergency,
non-finite decision/action value, or rolling-B3 failure.  Explicitly
not-applicable diagnostic fields are audited separately.  A case-level permit
must bind the
protocol digest, scene digest/config snapshot digest, seed, window, ordered
arms, cache manifest, and source commit.  The v5 geometry contracts remain
calibration-only and are not edited to set `trackingOutcomeAuthorized=true`.

Passing source-only preflight may authorize paired development tracking.  It
does not authorize X36, held-out seeds, GNN training, or a paper-level
generalization claim.
