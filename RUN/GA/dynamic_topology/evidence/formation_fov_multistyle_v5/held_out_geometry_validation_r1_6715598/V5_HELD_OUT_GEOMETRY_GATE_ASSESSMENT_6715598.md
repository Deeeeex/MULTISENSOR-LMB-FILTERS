# v5 Held-out Geometry Gate Assessment

## Decision

**GO for geometry eligibility; NO-GO for tracking claims.**

The frozen `formation-fov-multistyle-v5` convoy and relay suites passed all
`80/80` absolute realization gates and all `40/40` paired M24/X36 gates on
the exact preregistered seed manifest.  No structural failure or gate failure
was recorded.  This result establishes that the two non-radial scene families
are safe, observable and sufficiently scale-matched for later routing
experiments.  It does not establish routing or tracking benefit and does not
authorize any tracking outcome.

## Frozen provenance

| Item | Value |
|:--|:--|
| Generation commit | `671559817b0583a10aa2d6e2e1956313a1acca3c` |
| Protocol SHA-256 | `ec631ed036d4e4693353583bdc5fe02d8c5825f7a9d9e6ae187fcb1d235f0e7a` |
| Validation seeds | `401, 409, 419, 431, 443, 457, 467, 479, 487, 499, 509, 521, 541, 557, 569, 577, 587, 599, 607, 617` |
| Formal report SHA-256 | `634aa845bc7cfbb78feee052071a27e0e1212a0e536b1d2eb4d660c0130378b2` |
| MAT evidence SHA-256 | `d16d56ec7b766986ee9026879d9345a43a88abe575db46648b2b7fc6508b63c4` |
| Execution log SHA-256 | `3f84068ea5604e020f217327fd3ba4f9da187eecc9a31d43fdc3d9c00504d7cf` |
| Tracking authorized | `false` |

## Absolute realization envelope

Ranges below are extrema over twenty held-out seeds per preset.  A single
value in a range column means the metric was constant over all seeds.

| Preset | Blackout | Focus blackout max | Worst target max | Longest max | Single | Multi | Focus load | Normalized handovers | Entropy min | Blockage min | Close min | Target separation min (m) | Sensor-target min (m) |
|:--|:--|--:|--:|--:|:--|:--|:--|:--|--:|--:|--:|--:|--:|
| M24 convoy | 0.0017–0.0070 | 0.0000 | 0.0250 | 4 | 0.5218–0.5584 | 0.4364–0.4730 | 4.5395–4.5751 | 1.3125–1.4375 | 0.9620 | 0.6563 | 1.0000 | 20.00 | 36.80 |
| X36 convoy | 0.0012–0.0048 | 0.0000 | 0.0250 | 4 | 0.4054–0.4455 | 0.5509–0.5923 | 4.7347–4.7650 | 1.4167–1.5833 | 0.9833 | 0.6563 | 1.0000 | 20.00 | 36.20 |
| M24 relay | 0.0039–0.0065 | 0.0026 | 0.0938 | 14 | 0.4247–0.4482 | 0.5457–0.5714 | 5.2131–5.2574 | 0.8125–0.8333 | 0.9947 | 0.6563 | 1.0000 | 12.34 | 38.78 |
| X36 relay | 0.0045–0.0071 | 0.0061 | 0.0938 | 8 | 0.3866–0.4086 | 0.5848–0.6080 | 4.9847–5.0263 | 0.7167–0.7333 | 0.9885 | 0.6563 | 1.0000 | 14.77 | 30.25 |

The corresponding convoy gates were blackout `<=0.025`, focus blackout
`<=0.010`, worst-target blackout `<=0.050`, longest blackout `<=8`, single
coverage `>=0.35`, multi coverage `>=0.40`, load `4.0–5.25`, normalized
handovers `>=1.0`, entropy `>=0.95`, blockage overlap `>=0.60`, close
encounters `>=0.80`, target separation `>=14 m`, and sensor-target separation
`>=30 m`.

The relay gates were blackout `<=0.010`, focus blackout `<=0.010`,
worst-target blackout `<=0.120`, longest blackout `<=14`, single coverage
`>=0.35`, multi coverage `>=0.50`, load `4.5–5.50`, normalized handovers
`>=0.65`, entropy `>=0.95`, blockage overlap `>=0.60`, close encounters
`>=0.80`, target separation `>=9 m`, and sensor-target separation `>=30 m`.

## Paired M24/X36 envelope

| Style | Metric | Frozen gate | Held-out worst case | Decision |
|:--|:--|:--|--:|:--:|
| Convoy | Global blackout difference | `<=0.010` | 0.0022 | Pass |
| Convoy | Focus blackout difference | `<=0.010` | 0.0000 | Pass |
| Convoy | Worst-target difference | `<=0.030` | 0.0063 | Pass |
| Convoy | Longest-blackout difference | `<=4` | 1 | Pass |
| Convoy | Single / multi difference | each `<=0.15` | 0.1260 / 0.1271 | Pass |
| Convoy | X36/M24 focus-load ratio | `0.95–1.10` | 1.0378–1.0450 | Pass |
| Convoy | Normalized-handover difference | `<=0.20` | 0.1875 | Pass |
| Convoy | Entropy / blockage difference | `<=0.03 / <=0.05` | 0.0215 / 0.0000 | Pass |
| Relay | Global blackout difference | `<=0.010` | 0.0019 | Pass |
| Relay | Focus blackout difference | `<=0.015` | 0.0054 | Pass |
| Relay | Worst-target difference | `<=0.050` | 0.0188 | Pass |
| Relay | Longest-blackout difference | `<=8` | 8 | Pass |
| Relay | Single / multi difference | each `<=0.08` | 0.0432 / 0.0437 | Pass |
| Relay | X36/M24 focus-load ratio | `0.85–1.15` | 0.9543–0.9595 | Pass |
| Relay | Normalized-handover difference | `<=0.15` | 0.1167 | Pass |
| Relay | Entropy / blockage difference | `<=0.03 / <=0.05` | 0.0071 / 0.0000 | Pass |

## Interpretation and residual risk

1. Convoy remains observable and balanced at X36 rather than becoming an
   effectively easier all-visible scene.  X36 has more multi-formation
   overlap, but the paired difference and sensor load remain within the
   preregistered scale envelope.
2. The relay lane repair generalizes across the held-out seeds: the previous
   long, target-specific range blind band does not return.  Worst-target
   blackout remains at or below `9.38%` and the longest blind interval remains
   at or below `14` steps.
3. Several valid margins are deliberately narrow.  M24 relay reaches the
   longest-blackout limit (`14`), the relay paired longest difference reaches
   its limit (`8`), and the minimum X36 relay sensor-target separation is
   `30.25 m`, only `0.25 m` above the frozen safety threshold.  These are not
   failures, but they forbid claims of broad geometric robustness beyond the
   validated contract.  Any future wider geometry distribution should use a
   new version and a new held-out manifest rather than weakening v5.
4. Crossing remains `stress-only-v5` and is not promoted by this result.

## Next authorized step

The geometry result permits preparation of a separate, source-frozen routing
and tracking protocol.  Before opening any tracking outcomes, the scenario
figure and experiment document should be updated to the v5 geometry and its
evidence boundary.  M24 may then be evaluated on radial, convoy and relay with
one fixed method.  X36 tracking remains conditional on the same method showing
headroom without scene-specific retuning.
