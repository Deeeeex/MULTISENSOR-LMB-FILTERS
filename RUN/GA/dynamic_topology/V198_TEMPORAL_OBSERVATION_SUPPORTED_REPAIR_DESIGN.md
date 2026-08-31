# V198 temporal observation-supported repair

## Why V197 stops at H=3

V197 correctly spends its first X36 token on F2, but cooldown expiry is not
evidence that another repair is valuable.  On the frozen H=8 trajectory it
later spends tokens on F3 and F5.  F2 improves formation RMSE by about 8.04
units; F3 worsens it by about 6.23 units, and F5 is RMSE-neutral while
worsening E-OSPA.  The final controller remains `-3.174%` RMSE versus static.

An observable replay isolates the missing distinction.  When label support is
aggregated over the current and immediately preceding local-update pages and
includes currently reachable cross-edge senders, the first F2 candidate
remains unsupported, whereas the later F3/F5 candidates do not.  The same
two-page rule preserves the useful M24 F4 action in the earlier replay.

## Frozen method

V198 changes only the token-spending certificate:

1. compute the ordinary online V99 omission proposal from the current state;
2. aggregate per-sensor label detection-association support over the current
   and preceding completed local-posterior pages;
3. evaluate set-entry risk using receiver-or-cross-sender support;
4. if the V197 token is available, release at most the single formation with
   maximum remaining unsupported-entry risk; otherwise abstain;
5. after a release, retain the frozen two-page cooldown.

Past posterior pages are sanitized through the same observable-context
boundary as the current posterior.  They contain no truth, measurements,
future pages, target trajectories, delivery uniforms or numeric formation
features.  The carrier graph, KLA weights, message builder and byte ledger are
unchanged.

## Decision sequence

- First run paired H=3 M24/X36 and require the known useful release sequences
  `{F4, empty, empty}` and `{F2, empty, empty}`.
- Only if both are preserved, run X36 seed-211 t=72 H=8.
- Advance to fresh seeds only if H=8 restores positive RMSE versus static while
  retaining positive E-OSPA, consensus and attempted-byte savings.
- Do not tune the support threshold, history depth, token count or cooldown on
  these opened anchors.
