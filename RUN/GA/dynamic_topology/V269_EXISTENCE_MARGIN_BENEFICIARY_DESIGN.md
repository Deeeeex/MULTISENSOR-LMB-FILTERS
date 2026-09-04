# V269 existence-margin beneficiary

V268 proves that preserving the original source density across the relay has
large localization value, but its final receiver is selected only because it
is aligned with the current V242 backbone.  In the opened `t=58` decision,
relay S13 can physically reach all six F4 sensors and all six frozen links
deliver.  The backbone-first rule chooses S22 with selected-label existence
`0.577`; the highest reachable existence is `0.655`.  The ensuing E-OSPA and
cardinality loss is confined to F4.

V269 changes only this final beneficiary rule.  Among currently physical
members of the already selected target formation, it chooses the receiver
with maximum current existence probability for the selected label.  Current
backbone alignment and then physical sensor identity are deterministic
tie-breakers.  This is a causal label-survival margin, not a truth-level
quality score and not a realized-delivery teacher.

The V268 source, relay, label, one-page prediction, no-retry rule, two-hop byte
charge, V266 source-share grid, V267 asymmetric eta guard, and full V242
posterior backbone remain frozen.  The paired M24 `t=57--73` screen passes only
under the same joint tracking, formation-safety, consistency, and positive
static-byte-saving gate.  Failure closes the single-final-beneficiary packet
family rather than authorizing a receiver/weight sweep.

## Paired result and temporal ambiguity

V269 selects S24 at `t=58` (existence `0.655`) and S20 at `t=60`
(existence `0.632`) instead of V268's backbone-aligned S22.  Relative to V242,
network RMSE improves by `16.162%` and F4 event RMSE by `43.436%`.  The
existence-margin rule reduces, but does not eliminate, the set-side cost:
network/F4 event E-OSPA regress by `0.219%/0.892%`, consistency by `0.287%`,
and mean absolute cardinality error rises from `9.9412` to `10.0196`.  Spliced
static-byte saving remains positive at `9.960%`.

The receiver diagnosis is therefore causal but partial.  V269 improves both
E-OSPA and RMSE through `t=59`; cardinality and E-OSPA first worsen at `t=60`,
the same page on which the second packet is fused.  The opened trace cannot
tell whether this is a two-page delayed effect of the first packet or
over-intervention from the repeated packet.  One final causal ablation is
justified: admit at most one source-preserving packet for the continuous
formation-label risk episode.  If the one-shot action still develops the same
set loss, the packet's spatial state intervention itself—not receiver margin
or repetition—is unsafe and the family should be closed.
