# V142 M24 identity checkpoint

## Decision

V142 passes the frozen M24 mechanism gate and preserves the complete V139 M24
candidate outcome exactly.  This authorizes the paired X36 mechanism screen.
It does not authorize a communication-method claim or a main-document result:
the exact relay-supported label state still comes from the charged dual
payload.

## Frozen result

- Source commit: `d927cfc`.
- Preset / seed / anchor / action: `m24-formation-fov` / `1601` / `95` / `25`.
- Intervention E-OSPA gain: `+5.614%`.
- Full-window / mature-window gain: `+0.511% / +0.000%`.
- Minimum sensor / formation gain: `-0.003% / +0.000%`.
- Exact reentry match: `100.000%`.
- Charged attempted-byte delta: `+89.182%`.
- Registered gate: passed.

## Identity and mechanism checks

The V142 and V139 M24 candidate outcomes are identical for every saved
sensor-time E-OSPA value (`maximum absolute difference = 0`).  They also have
identical selected routes, nominal weights, attempted and delivered edges,
abstention masks, attempted bytes, delivered bytes and auxiliary bytes.

V142 requested four spatially supported augmentations and appended six
exact-relay labels in total, with no insufficient-reference fallback.  The
four affected cells were sensors 10--12 on page 5 and sensor 11 on page 6.
Every cell was subsequently overwritten by the inherited V139 predictive
output fallback.  The registered M24 behavior is therefore exact no-harm, not
an additional gain.

## Reporting boundary and next step

This checkpoint remains a repository experiment record.  The only authorized
next action is the frozen X36 V142 screen at action `51`, using the same seed,
route, delivery realization, communication accounting and gates.  A passing
X36 result would establish only that a small set of relay-supported
label--state corrections contains the missing estimation information.  The
reference state must still be replaced by observable, single-payload
edge--label decisions before the mechanism can become a deployable method or
enter the main Lark progress document.
