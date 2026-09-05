# Next discriminating diagnostic: locate existence loss

Status: first pass completed on 2026-09-06. See
`evidence/tracking_aligned_v281/m24_existence_loss_seed1301/EXISTENCE_LOSS_LOCALIZATION_V281.md`
and `V281_EXISTENCE_LOSS_LOCALIZATION_FINDING.md`. The design below was
executed only on the three saved M24 anchors; no full filter or X36 trace ran.

## Decision

V278 is complete. Self fallback improves X36 set error and consistency over
V242, but loses 4.836% conditional RMSE and fails its frozen follow-up gate.
No M24 fallback run or weight sweep follows. V279 rules out large gains from
position refinement alone. V280 finds little global geometric blackout but
substantial finite-horizon propagation delay, particularly at X36.

The remaining question is where target-existence information is lost. Do not
equate an ideal geometric source path with a label in an actual input packet.

## Bounded first pass

Reuse the existing V250 M24 behavior cache at opened anchors 70, 84 and 151,
located through the saved oracle's `cachePath`. Examine only the V242
reference route, not candidate gateway substitutions or a fitted selector.
These snapshots can localize a one-round mechanism; they cannot establish
full-episode or X36 causality.

Use existing `captureLabelKlaDiagnosticsEnabled` support in
`fuseLmbPosteriorsByLabel`; inspect the exact runtime collection/extraction
rules before constructing an offline input. Keep unavailable packets,
empty posteriors and absent individual labels separate. Do not replace
MAP-cardinality extraction by an unverified per-label 0.5 threshold.

For each receiver and label, retain input existence, actual active weights,
weighted input log odds, spatial log normalizer, fused existence and the
post-pruning/output decision. Separate:

1. no label-bearing input among the actual received packets;
2. inputs present, but their weighted existence evidence is already weak;
3. inputs present, with additional existence loss from density overlap;
4. fused label retained internally, but absent from the extracted target set.

Report counts and existence mass as distributions by receiver/formation,
not a pooled safety score. A truth-independent supported label need not be
a true target. If truth matching is used for offline interpretation, identify
it explicitly and never feed it to the routing or fusion policy.

## Stopping and escalation

This is not a repeat of V273's attempt to predict candidate H=3 harm: V273
found no stable scalar safety signal. The present readout instead attributes
the reference operator's immediate loss terms and asks which mechanism
deserves intervention. Do not train a GNN or optimize thresholds on these
three anchors.

If the cached snapshots lack the required local-update/received-input state,
state the gap before considering a targeted observer-enabled rerun. Do not
manufacture a runtime history from aggregate metrics. If the mechanism is
clear only on M24, an X36 stage trace is still needed before a scale-general
method claim. Any subsequent policy needs its own paired test against the
unchanged static and sparse references.
