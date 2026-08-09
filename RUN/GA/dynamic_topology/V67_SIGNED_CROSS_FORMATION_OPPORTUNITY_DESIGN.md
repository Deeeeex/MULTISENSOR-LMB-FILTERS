# V67 signed cross-formation opportunity diagnostic

The V66 relay scan found no state in which withholding incoming
cross-formation posterior bundles reached the frozen 1% rescue-risk budget.
That result can mean either that the relay trajectory has no material dynamic
headroom or that its useful action has the opposite sign.

V67 separates the two current-only quantities that V65 already evaluates with
one network denominator:

- **quarantine pressure** is receiver-supported existence restored when an
  incoming cross-formation bundle is withheld;
- **transport-retention pressure** is sender-supported existence lost by the
  same withholding counterfactual.

Their signed difference is transport minus quarantine.  A material positive
value does not prove that a new route improves tracking, but it does establish
that a suppression-only action removes more currently supported information
than it protects.  A material negative value supports the existing quarantine
action.  Both below 1% means the state is neutral under this action family.

The first diagnostic reuses the 25 truth-free relay caches from seed 1301.  It
does not authorize a route, choose a tracking time, read E-OSPA, or train a
model.  Its only method decision is whether the next action space needs a
positive transport branch in addition to V65/V66 quarantine.
