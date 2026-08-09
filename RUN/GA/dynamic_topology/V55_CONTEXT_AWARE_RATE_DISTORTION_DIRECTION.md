# V55 context-aware label rate-distortion direction

The paper-facing derivation, exactness boundary, and promotion gates are
collected in `V55_FULL_REFERENCE_RATE_DISTORTION_THEORY.md`.

## Why this direction exists

The current V54 offline teacher evaluates a receiver and its selectable
cross-formation residual senders.  The actual runtime subsequently fuses the
same receiver with fixed V46 backbone and nonselective residual inputs as
well.  Because Bernoulli KLA includes normalization, adding identical fixed
sources to every candidate can still change candidate distortion, option
ordering, and whether the receiver log-odds floor becomes active.

V55 is the direct correction if the frozen V54 X36 gate fails with useful
communication saving.  It does not change the V46 route.  It changes the
teacher from a selective-source approximation into a rate-distortion problem
conditioned on the full fixed fusion context.

## Context-aware reference and candidate

For receiver `j` and label `ell`, split the current fusion sources into:

- `F_j`: fixed V46 inputs, including the dominant backbone and all
  nonselective residual inputs;
- `S_j,ell`: every present selectable cross-residual input that the full V46
  message would supply for label `ell`;
- `A_j,ell`: the selected subset of `S_j,ell`.

The full safe reference and one candidate are

`q_ref(j,ell) = KLA(q_receiver, F_j, S_j,ell)`,

`q_A(j,ell) = KLA(q_receiver, F_j, A_j,ell)`.

Every candidate contains exactly the same fixed sources.  The only decision
is which selectable label payloads are added.  This makes the teacher's
distortion match the nominal runtime fusion context instead of evaluating the
residual subproblem in isolation.

Unlike V54, V55 does not use evidence type to remove unsupported-absence or
ambiguous label objects from the reference.  Those objects may still carry a
valid predicted track even when the current measurement offers no direct
support.  Treating their evidence category as permission to delete them would
build a positive-evidence bias into the teacher.  Evidence remains available
as a feature for a later deployable policy and is used to decide whether the
optional receiver protection applies, but the selection-only rate-distortion
reference is the actual full V46 label input set.

## Rate-distortion objective

For matched LMB label spaces, the LMB KLD decomposes into Bernoulli label
terms.  The teacher objective can therefore be written as

`min_A sum_ell D_KL(q_ref(j,ell) || q_A(j,ell))`

subject to

`B_synopsis + B_headers(A) + sum_ell B_label(A_j,ell) <= B_full`.

The label payload costs are additive.  An edge header is paid only once when
at least one label activates that edge.  For bounded receiver in-degree, the
existing dynamic program over `(label index, active-edge mask, bytes)` remains
an exact solver for the enumerated option table.  The theoretical claim is
thus an exact offline rate-distortion teacher for the installed LMB-KLA
consumer and byte model, not an exact optimizer of tracking truth.

The spatial part of each option still uses numerical GM KLD approximation, so
the paper must distinguish exact combinatorial selection from approximate
option distortion.  No target truth enters this teacher.

## Receiver protection

For labels with current positive receiver evidence, each candidate is mapped
to the feasible log-odds set before its distortion is compared.  The final
existence-only boundary correction is the KL-nearest constrained Bernoulli
projection derived in `V54_CONSTRAINED_BERNOULLI_PROJECTION.md`.

This projection should be represented inside the option table, not applied
only after the teacher has ranked unprojected candidates.  Greedy removal of
already received selective inputs may remain a causal fallback, but it is not
part of the exact rate-distortion guarantee.

## Causal online protocol

A deployable version requires an explicit two-stage communication period.

1. Fixed V46 backbone payloads and compact residual-label synopses are
   delivered to the receiver.
2. The receiver constructs its fixed fusion context and scores residual label
   subsets using only received information and link history.
3. The receiver requests the selected GM labels; senders return those labels
   within the remaining period budget.
4. The receiver performs context-aware fusion and the constrained Bernoulli
   correction.

This protocol adds a request round and possible latency.  If the simulator
continues to model all stages as one instantaneous step, the paper must state
that one fusion period contains these subrounds and include the request bytes
in final accounting.  A one-shot sender policy would need a different feature
contract because a sender does not possess the receiver's complete fixed
fusion context.

## Learned approximation

The set GNN should approximate option distortion or option ranking conditioned
on a receiver-side summary of the fixed fused context.  Its input is no longer
only receiver and sender synopses.  It also needs:

- the receiver label after fixed-context fusion;
- disagreement between each selectable sender and that fixed-context label;
- fixed-context source count, total fusion weight, staleness, and reliability;
- the same compact evidence and mixture-complexity features already defined
  for V54.

Target truth, future measurements, and future delivery outcomes remain
forbidden online inputs.

## Required attribution experiment

Before GNN training, one paired scene must separate three effects:

| Arm | Full/selective residual payload | Receiver projection | Question |
|:--|:--|:--|:--|
| projection-only | full | on | Does the safety rule itself improve or bias cardinality? |
| selection-only | context-aware selective | off | Does rate-distortion compression preserve tracking without the clamp? |
| combined | context-aware selective | on | Is there positive interaction between compression and protection? |

If projection-only explains nearly all tracking gain, the contribution is a
robust fusion correction rather than learned routing.  If selection-only
preserves tracking and saves bytes while projection hurts cardinality, the
floor must be weakened or removed.  Only a stable combined advantage supports
the full receiver-safe learned-routing story.

## Decision rule after V54

- V54 clears all frozen X36 gates: run the attribution arms before exporting
  GNN training data.
- V54 misses tracking but saves substantial bytes: implement V55 context-aware
  options and the attribution arms; do not train on V54 targets.
- V54 misses both tracking and byte saving: abandon this payload-selection
  branch and return to topology-level method design.

The frozen X36 convoy result follows the second branch: V54 saved 40.25% of
total attempted bytes and 50.96% on the selective path, but full-horizon
E-OSPA, focus-window E-OSPA, and cardinality error changed by -1.91%, -2.64%,
and -6.22%, respectively.  The 6,356 existence clamps show that projection
was a dominant intervention.  GNN training is therefore stopped; V55 must
first separate projection-only, full-reference selection-only, and combined
behavior.
