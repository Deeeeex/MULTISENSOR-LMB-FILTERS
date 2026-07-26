# Directed reliability dynamicity audit

- Generated: 2026-07-26 08:36:38
- Seeds: `[7 17 27]`
- All routes static: `1`
- Exact fixed-index clones: `1`

| Preset | Seed | Sensors | Steps | Distinct maps | Within-window receiver changes | Exact fixed clone | Cross-formation | Coverage | Max sender load | First formation sender map |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--|
| m24-hard | 7 | 24 | 160 | 1 | 0.0000 | 1 | 0.0000 | 1.0000 | 5 | `[2 1 1 1 1 1]` |
| x36-clean-scale | 7 | 36 | 160 | 1 | 0.0000 | 1 | 0.0000 | 1.0000 | 5 | `[2 1 1 1 1 1]` |
| m24-hard | 17 | 24 | 160 | 1 | 0.0000 | 1 | 0.0000 | 1.0000 | 5 | `[2 1 1 1 1 1]` |
| x36-clean-scale | 17 | 36 | 160 | 1 | 0.0000 | 1 | 0.0000 | 1.0000 | 5 | `[2 1 1 1 1 1]` |
| m24-hard | 27 | 24 | 160 | 1 | 0.0000 | 1 | 0.0000 | 1.0000 | 5 | `[2 1 1 1 1 1]` |
| x36-clean-scale | 27 | 36 | 160 | 1 | 0.0000 | 1 | 0.0000 | 1.0000 | 5 | `[2 1 1 1 1 1]` |

Interpretation: the audited reliability arm is a fixed receiver-specific one-incoming-message control. Any tracking gain from that arm cannot be attributed to online topology adaptation.
