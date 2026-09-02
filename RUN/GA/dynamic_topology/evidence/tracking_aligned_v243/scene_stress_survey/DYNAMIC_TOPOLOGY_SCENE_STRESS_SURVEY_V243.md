# V243 dynamic-topology scene stress survey

- Seed: `1301`
- Tracking outcome authorized: `0`
- Evidence boundary: V243 uses geometry and physical communication reachability only. It selects candidate scene families for later paired tracking experiments but does not establish tracking, consistency, communication-byte, or generalization benefit.

| Preset | Style | Valid | Formation connected | Initial tree feasible | First tree failure | Failure episodes | Exact static route physical | First exact-route failure | Qualification |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|:--|
| m24-formation-fov-convoy | parallel-convoy | 1 | 1.000 | 1.000 | -- | 0 | 1.000 | -- | static-route-control |
| x36-formation-fov-convoy | parallel-convoy | 1 | 1.000 | 1.000 | -- | 0 | 1.000 | -- | static-route-control |
| m24-formation-fov-relay | linear-relay | 1 | 1.000 | 1.000 | -- | 0 | 1.000 | -- | static-route-control |
| x36-formation-fov-relay | linear-relay | 1 | 1.000 | 1.000 | -- | 0 | 1.000 | -- | static-route-control |
| m24-formation-fov-crossing | orthogonal-crossing | 1 | 1.000 | 1.000 | -- | 0 | 0.950 | 153 | dynamic-gateway-candidate |
| x36-formation-fov-crossing | orthogonal-crossing | 1 | 1.000 | 0.981 | 158 | 1 | 0.863 | 139 | dynamic-tree-candidate |
| m24-formation-fov-merge-split | merge-split | 1 | 1.000 | 1.000 | -- | 0 | 1.000 | -- | static-route-control |
| x36-formation-fov-merge-split | merge-split | 1 | 1.000 | 1.000 | -- | 0 | 1.000 | -- | static-route-control |
| m24-formation-fov-target-overlap | target-group-overlap-split | 1 | 1.000 | 1.000 | -- | 0 | 1.000 | -- | static-route-control |
| x36-formation-fov-target-overlap | target-group-overlap-split | 1 | 1.000 | 1.000 | -- | 0 | 1.000 | -- | static-route-control |
| m24-formation-fov-curved-corridor | curved-corridor | 1 | 1.000 | 1.000 | -- | 0 | 1.000 | -- | static-route-control |
| x36-formation-fov-curved-corridor | curved-corridor | 1 | 1.000 | 1.000 | -- | 0 | 1.000 | -- | static-route-control |
| m24-formation-fov-braided-handover | braided-handover | 1 | 1.000 | 1.000 | -- | 0 | 1.000 | -- | static-route-control |
| x36-formation-fov-braided-handover | braided-handover | 1 | 1.000 | 1.000 | -- | 0 | 1.000 | -- | static-route-control |
| m24-formation-fov-formation-braid | formation-braid | 1 | 1.000 | 0.431 | 70 | 1 | 0.331 | 54 | dynamic-tree-candidate |
| x36-formation-fov-formation-braid | formation-braid | 1 | 1.000 | 0.338 | 55 | 1 | 0.300 | 49 | dynamic-tree-candidate |

## Decision rule

- `dynamic-tree-candidate`: the physical formation graph stays connected, but the initially registered formation tree fails.
- `dynamic-gateway-candidate`: the formation tree remains feasible, but one or more registered sensor gateway edges fail and must be reassigned.
- `static-route-control`: useful for cross-style generalization, but it cannot establish the value of dynamic repair.
- `physical-formation-disconnection`: requires store/forward or intermittent-connectivity semantics beyond the current method.
- `scene-repair-required`: geometry or validation must be fixed before any tracking comparison.
