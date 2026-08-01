# Formation-FoV Multistyle Suite v1: Figure QA

## Registered output

- Renderer: Python 3 with Matplotlib; Octave is used only to export tested
  scenario geometry as JSON.
- Source: `source/formation_fov_multistyle_suite_seed41.json`.
- Snapshot: seed 41, time 80.
- Canvas: `7.2 x 5.15 in` (`518.4 x 370.8 pt`).
- Deliverables: editable SVG, one-page PDF, and 300-dpi PNG
  (`2160 x 1545 px`).
- Outcome boundary: no posterior, tracking result, learned score, or target
  outcome is consumed by this deterministic scene schematic.

## Structural checks

- `test_multistyle_formation_scenarios` passes for all six M24/X36 presets.
- Every scene keeps the registered `120 deg / 300 m` sensing envelope, one
  shared boresight per formation, finite headings, motion limits, separation
  limits, graph budgets, and an all-time physical static backbone.
- The D12, M24, X36, and X48 legacy formation-FoV presets produce the same
  static-edge fingerprints as parent commit `c32a064`:

| Preset | SHA-256 of ordered static edge list |
|---|---|
| `d12-formation-fov` | `58ce14b7c8391b2f2602410867a9975c0d2fa485bcf75362801e6671c367b933` |
| `m24-formation-fov` | `b13e87866f59ed991c42fb3551b036b6439728f78b4a15d232ba23be17c4e8bd` |
| `x36-formation-fov` | `a2710c6d46dd7633160971b454f5fee6e7c993f404071374ac9d2f09109858a0` |
| `x48-formation-fov` | `8a02d0adff85accb82d31ef7d9a3272f827ef9f0fad5b55311a7f3cf3c7a853c` |

## Vector and typography checks

- SVG contains 112 editable `<text>` elements and zero `<image>` elements.
- PDF uses embedded Arial and Arial Bold Type-0 fonts.
- Lines, markers, FoV sectors, links, and paths remain vector objects in SVG
  and PDF; the PNG is a secondary display copy only.
- `git diff --check` and Python byte-code compilation pass.

## Visual inspection

- The same metre-scale axes are used in all six panels, so FoV range and
  formation spacing are directly comparable.
- X36 occupies the top row and M24 the bottom row; columns consistently map to
  parallel convoy, orthogonal crossing, and linear relay.
- Sensor/target snapshots, paths, exact sensor-origin FoV sectors, and static
  formation links remain visually distinguishable without label overlap or
  clipping.
- One shared legend is used; no panel repeats decorative labels or introduces
  a formation-level FoV that is absent from the simulator.

## File fingerprints

| File | SHA-256 |
|---|---|
| `formation_fov_multistyle_suite_v1.svg` | `1d8c83f888b4489c39ca6024b11bb906b83b25b1204b877a453ce17bd9ac24bd` |
| `formation_fov_multistyle_suite_v1.pdf` | `ada8e7f03d6e7ea45d2f6de9edb44d34b8f3f91181743cd75f78f74dc6d8f9c6` |
| `formation_fov_multistyle_suite_v1.png` | `98ce0db62b14d236ca29c016371fbb4abd4345b8a102262c830583a615eeb4f4` |
| `formation_fov_multistyle_suite_seed41.json` | `218fb4ef5219dd8a8075ee397f900bea56c5af44a6f95d9a5e3a6294cfded319` |

These checks authorize the figure as a geometry/protocol illustration only.
The new styles remain calibration-only until their difficulty gates are
frozen and tracking performance is evaluated on unopened seeds.
