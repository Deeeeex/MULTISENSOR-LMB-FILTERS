# V145 X36 closure: delayed W/R benefit does not transfer

## Decision

V145 fails the separately frozen X36 delayed-horizon transfer audit.  The
observable protection-load gate retains a short-window improvement, but the
effect reverses over the full and mature windows, creates large weakest-node
and weakest-formation losses, and slightly increases attempted bytes.  The
result closes hand-designed whole-posterior W/R temporal gating rather than
opening fresh-seed validation.

This conclusion does not revise the original V145 M24 gate.  M24 already
failed its registered intervention threshold; the X36 audit asked only
whether M24's delayed benefit was a transferable mechanism.  It is not.

## Frozen paired result

| Scale | Intervention | Full window | Mature window | Min. sensor | Min. formation | Byte delta | Reentry |
|:--|--:|--:|--:|--:|--:|--:|--:|
| M24 | +3.060% | +6.221% | +6.938% | +1.564% | +3.335% | -0.117% | 100% |
| X36 | +3.819% | -0.841% | -1.367% | -6.191% | -5.296% | +0.202% | 100% |

- Source commit: `c5dcb28`.
- X36 preset / seed / anchor / action:
  `x36-formation-fov / 1601 / 80 / 51`.
- X36 wire roles: `4563` reference and `297` working payloads; every
  opportunity carried one complete mixture-aware LMB posterior and auxiliary
  attempted bytes remained zero.
- The output-only predictive fallback fired at `107` node--times, compared
  with `33` for M24.
- The existing registered mechanism gate and the delayed-transfer gate both
  fail.

## What the scale contrast resolves

The M24 full-window gain was not evidence for a scale-stable temporal
mechanism.  At X36, the longer protected prefix and larger number of working
payload roles leave some nodes and formations with a persistent lineage
error even though the relay state rejoins exactly.  Exact internal reentry is
therefore insufficient: the downstream tracking recursion has already
consumed the harmful intermediate outputs.

Another cadence, protection-fraction threshold, or all-R latch time would be
post-outcome tuning of the same closed action family.  Do not train a GNN to
imitate or refine this whole-posterior gate.

## Next bounded decision

Run the already frozen V149 X36 reference-cover diagnostic once.  It tests a
different, X36-specific mechanism identified before this outcome: whether a
single hybrid payload can retain the rare reference-supported label state
that repaired six severe V142 output cells.  V149 remains privileged and
cannot reopen its failed M24 gate.

If V149 also fails, close W/R label-role multiplexing and move to an explicit
label-effective fusion action:

1. omission is transmitted as deliberate label-wise abstention and receives
   exactly zero KLA weight, rather than being inferred through the
   `fov-aware-censored` approximation;
2. every selected object remains a complete mixture-aware Bernoulli density;
3. the learned or analytic score targets the final readout and recursive
   finite-horizon tracking outcome, not an intermediate virtual fusion; and
4. a deterministic projector enforces the byte cap and time-expanded
   label-wise information-flow graph.

Per-label source selection and FoV-based active sensor sets are prior art.
The remaining research question is the communication-constrained,
time-varying edge--label graph under KLA and downstream tracking risk.

