# Formation H=3 first-step mode-vector probe v22

## Question

v21 finds a strong near-safe terminal vector, but the final step cannot repay
the consensus debt created by the fixed `[9,13]` prefix.  v22 therefore moves
the heterogeneous decision to the first step and asks whether a coordinated
initial action can create tracking value without creating irrecoverable debt.

## Frozen probe

- opened state: `m24-formation-fov / seed 211 / t=72`;
- center: all-reference mode vector `[1,1,1,1]`;
- first-step candidates: the complete Hamming-radius-two neighborhood of the
  center, containing 67 vectors;
- second and third steps: registered reference;
- full teacher dictionary: the same 256-vector bank frozen in v21;
- reproduction control: `[1,1,3,1] -> reference -> reference` must reproduce
  the v17 mean-only ceiling within `5e-6` percentage points;
- strict feasibility: all six targets nonnegative;
- strong headroom: strict feasible and at least `3%` mean tracking gain;
- physical, payload, exact-execution, rolling-B3, truth, repair, emergency,
  and infeasibility gates remain unchanged.

The candidate set includes every singleton mode change and every two-formation
combination across all three dynamic modes.  It is frozen before any v22
outcome is observed.

## Decision boundary

A strong safe candidate would directly supply a one-step teacher label.  If
only weak safe candidates exist, their outcomes will define debt-aware beams
for a pre-registered two-step expansion.  If no nonreference action is safe,
the next beam must explicitly trade early mean gain against a bounded
consensus-debt budget rather than filtering only on terminal feasibility.

This remains a privileged single-state mechanism probe.  Seeds 223/227, X36,
and final seeds remain unopened.
