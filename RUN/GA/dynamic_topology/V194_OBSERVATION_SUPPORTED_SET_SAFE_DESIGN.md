# V194: observation-supported set-safe posterior omission

## Problem

V99 saves communication by withholding cross-formation full posteriors for
receiver formations with positive current-state value.  The M24/X36 teachers
show that this is useful on average but can push the downstream LMB extraction
into unsupported over-cardinality.  The old per-label 0.5 crossing guard does
not see labels that enter because the MAP cardinality itself increases.

## Online policy

At every fusion page, V194 performs four current-observable steps:

1. construct the ordinary V99 positive-net omission proposal;
2. for each proposed formation, marginalize its current link-outcome
   counterfactual into an LMB cardinality PMF and MAP extraction;
3. inspect labels entering the candidate extraction and remove the formation
   from the proposal if any entering label has measurement-association mass
   below the registered positive-support threshold;
4. execute the projected omission set through the unchanged
   formation-conditioned message builder.

Removing a formation restores the normal full-posterior path.  It does not add
a side channel or modify topology, KLA weights, message serialization or byte
accounting.  The rule contains no time, formation or label identifier and is
recomputed on the state reached after every earlier action.

## Safety structure

For a reference cardinality PMF with unique MAP margin `Delta`, a candidate
PMF perturbation bounded by `delta` in infinity norm preserves the MAP
cardinality whenever `delta < Delta/2`.  V194 retains this sufficient
certificate as a risk diagnostic, but it does not release every uncertified
action: V192 showed that supported recovery can legitimately change the MAP
cardinality.  The executable projection instead releases only unsupported
entries.  Full posterior is the conservative fallback.

## First paired pilots

| Scale | Frozen anchor | Expected first-page V99 proposal | Expected V194 releases |
|:--|:--|:--|:--|
| M24 | seed 211, t=104, H=3 | F1 + F3 + F4 | F4 |
| X36 | seed 211, t=72, H=3 | F1 + F2 + F4 + F5 | F2 + F5 |

These expectations come from observable V193 analysis.  Runtime policy
diagnostics, not fixed plans, determine the executed releases.  The first
decision is whether the joint online policy preserves positive E-OSPA,
consensus and byte savings while turning mean RMSE positive on both scales.
The existing strict development gate remains unchanged.

## Evidence boundary

The M24/X36 anchors and support rule were developed from opened seed-211
states.  V194 results are method-development evidence until the policy is
frozen and evaluated on new seeds and the convoy, relay, merge-split and curved
geometries.

