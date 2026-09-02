# V244 information-coupled formation-braid preflight

- Seed: `1301`
- Coupled-scene gate passed: `1`
- Original uncoupled failure demonstrated: `1`
- Tracking outcome authorized: `0`
- Evidence boundary: V244 checks scene validity, physical formation connectivity, initial-tree failure, and whether registered target handoffs cross the affected tree cuts. It does not establish a tracking, consistency, communication-byte, or generalization benefit.

| Preset | Valid | Connected | Initial tree feasible | First failure | Failed cuts | Failed cuts with target transfer | All tree cuts covered | Min transfers / failed cut |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|
| m24-formation-fov-formation-braid | 1 | 1.000 | 0.431 | 70 | 1 | 0 | 0 | 0 |
| m24-formation-fov-coupled-formation-braid | 1 | 1.000 | 0.431 | 70 | 1 | 1 | 1 | 1 |
| x36-formation-fov-formation-braid | 1 | 1.000 | 0.338 | 55 | 2 | 0 | 0 | 0 |
| x36-formation-fov-coupled-formation-braid | 1 | 1.000 | 0.338 | 55 | 2 | 2 | 1 | 1 |
| x48-formation-fov-formation-braid | 1 | 1.000 | 0.281 | 46 | 3 | 0 | 0 | 0 |
| x48-formation-fov-coupled-formation-braid | 1 | 1.000 | 0.281 | 46 | 3 | 3 | 1 | 1 |

## Per-scene cut evidence

### m24-formation-fov-formation-braid

- Initial tree edges: `[1 2;2 3;3 4]`
- Edge first-failure times: `[NaN 70 NaN]`
- Target source/destination pairs: `[1 2;2 1;3 4;4 3]`
- Target transfers across each tree cut: `[2 0 2]`

### m24-formation-fov-coupled-formation-braid

- Initial tree edges: `[1 2;2 3;3 4]`
- Edge first-failure times: `[NaN 70 NaN]`
- Target source/destination pairs: `[1 2;2 3;3 4;4 3]`
- Target transfers across each tree cut: `[1 1 2]`

### x36-formation-fov-formation-braid

- Initial tree edges: `[1 2;2 3;3 4;4 5;5 6]`
- Edge first-failure times: `[NaN 55 NaN 95 NaN]`
- Target source/destination pairs: `[1 2;2 1;3 4;4 3;5 6;6 5]`
- Target transfers across each tree cut: `[2 0 2 0 2]`

### x36-formation-fov-coupled-formation-braid

- Initial tree edges: `[1 2;2 3;3 4;4 5;5 6]`
- Edge first-failure times: `[NaN 55 NaN 95 NaN]`
- Target source/destination pairs: `[1 2;2 3;3 4;4 5;5 6;6 5]`
- Target transfers across each tree cut: `[1 1 1 1 2]`

### x48-formation-fov-formation-braid

- Initial tree edges: `[1 2;2 3;3 4;4 5;5 6;6 7;7 8]`
- Edge first-failure times: `[NaN 46 NaN 71 NaN 101 NaN]`
- Target source/destination pairs: `[1 2;2 1;3 4;4 3;5 6;6 5;7 8;8 7]`
- Target transfers across each tree cut: `[2 0 2 0 2 0 2]`

### x48-formation-fov-coupled-formation-braid

- Initial tree edges: `[1 2;2 3;3 4;4 5;5 6;6 7;7 8]`
- Edge first-failure times: `[NaN 46 NaN 71 NaN 101 NaN]`
- Target source/destination pairs: `[1 2;2 3;3 4;4 5;5 6;6 7;7 8;8 7]`
- Target transfers across each tree cut: `[1 1 1 1 1 1 2]`
