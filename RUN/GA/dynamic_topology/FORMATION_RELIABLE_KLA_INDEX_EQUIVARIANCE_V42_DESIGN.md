# V42 formation-route index-equivariance diagnostic

## Question

V41 found that suppressing the input bundle of physical formation F6 reduced
the exact mean-square KLA propagation factor in all four X36 seeds, while no
M24 formation improved it.  The residual route is built by a deterministic
Euler walk, however, and that implementation starts from the first formation
in stable array order and resolves graph neighbors by their array index.

V42 asks whether the apparent F6 value survives a change that has no physical
meaning: reorder complete formation blocks in node coordinates, while keeping
every formation label, member, geometry page, link reliability, and route
history physically unchanged.

## Competing explanations

- **Physical-role explanation:** the same physical formation remains best
  after every coordinate permutation, while its position in the array changes.
- **Order-position explanation:** the best action moves between physical
  formations but stays at the same position in the reordered formation list.
- **General route-order confounding:** restored reference or candidate routes
  change, but the best action follows neither simple pattern.

Any change in a restored physical route is already a failure of index
equivariance.  A stable best label alone cannot repair that failure because the
candidate action is then being evaluated against a different arbitrary route.

## Frozen transformation

For each registered preset and seed, let the original stable formation order
be `g = [g1,...,gF]`.  V42 evaluates all `F` cyclic orders.  Each order moves a
complete formation block as one unit; it never changes labels or membership.
Every node-indexed object is permuted on both receiver and sender axes:

- registered and current physical adjacency;
- the two-page selected-route history;
- sender-row/receiver-column drop probabilities;
- scene static adjacency and local posterior slots.

Posterior slots are permuted only as opaque cells and their contents are never
read.  The policy output is mapped back to the original node coordinates before
comparison.

## Pass conditions

For every shift, compared by unchanged physical formation label:

1. restored reference adjacency is exactly equal to the baseline;
2. restored candidate adjacency and availability are exactly equal;
3. restored fusion weights differ by at most `1e-12`;
4. adaptive horizon is identical;
5. exact mean-square factors differ by at most `1e-12`.

All conditions must hold.  The test has no statistical tolerance because the
inputs differ only by a finite coordinate permutation.

This first diagnostic does not enumerate the full permutation group and does
not permute sensors within a formation.  It covers `4/24` block orders on M24
and `6/720` on X36.  A pass therefore means only that the tested cyclic
block-order confound was not detected; it never authorizes a physical-action
interpretation.  Full block permutations and within-formation sensor
permutations remain mandatory follow-up audits.

## Registered batch

- Presets: `m24-formation-fov`, `x36-formation-fov`.
- Seeds: `41`, `43`, `47`, `53`.
- Permutations: all cyclic complete-formation-block orders.
- Metric: operational `renormalize` exact expected centered-L2 factor.
- Horizon: V41 adaptive reference horizon targeting `rho <= 0.90`.

The registered conclusion requires the complete `2 x 4 = 8` Cartesian batch.
Preset/seed subsets are allowed only for smoke testing and must carry
`exploratorySubsetOnly=true`.

## Claim boundary

This is a geometry-only development audit.  It materializes the planned sensor
and link-probability schedules and does not satisfy the formal runtime-input
boundary.  It reads no target, measurement, posterior content, realized link
uniform, or tracking outcome.

- Failure invalidates a physical interpretation of the V41 F6 signal and
  requires an index-equivariant residual-route construction before further
  controller development.
- Passing only removes this tested cyclic block-order confound.  It does not
  authorize a physical-formation interpretation, turn the structural factor
  into a tracking-risk guarantee, or authorize M24/X36 experiments.
