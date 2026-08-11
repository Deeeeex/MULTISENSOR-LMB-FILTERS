# V96 addressable-risk adaptive payload control

## First-principles correction

V95 showed that moving a fixed 0.05 residual weight has almost no control
authority on M24 and only a 0.252% effect on X36. V94, by contrast, improved
the two M24 anchors by 6.188% and 7.206% by temporarily withholding complete
cross-formation posteriors at receiver formations whose local observations
would otherwise be suppressed. The useful control variable is therefore not
the visual identity of an edge alone, but whether a complete posterior should
participate in the effective label-wise fusion graph.

The V94 selector divided rescue mass by whole-network reference mass and then
required at least 1% risk. That fraction shrinks as unrelated sensors and
labels are added: the two X36 anchors reached 0.907% and 0.964% and therefore
never acted. At X36 t100, the selector also counted risk belonging to a
formation that could not be safely suppressed in its 80% coverage denominator,
making a safe cover mathematically impossible.

## V96 decision rule

For each receiver formation, V96 predicts the current label-existence change
caused by withholding complete cross-formation payload while retaining the
lightweight control synopsis. A formation is addressable only when this
counterfactual produces no downward decision crossing for a label supported
by an active cross-formation sender.

V96 then selects the smallest subset that:

1. covers at least 80% of positive rescue mass among addressable formations;
2. has cross-supported useful-loss mass no larger than its rescue mass; and
3. contains only currently available, finite and downward-crossing-free
   actions.

There is no threshold divided by total node count or total network reference
mass. If no positive addressable risk exists, the method returns the full
payload static reference.

## Matched development arms

Every anchor freezes three arms before reading tracking truth:

1. static carrier graph with full payload;
2. the V96 selected formation set active for one step, then full payload;
3. the same selected set active for all three steps.

All arms keep the same physical carrier graph, fusion weights, cached
posterior, measurements, link uniforms, filter RNG and communication
constraints. The persistent arm must improve mean E-OSPA by at least 5% on
every M24 and X36 anchor, must not regress the worst sensor, weakest formation,
consensus or attempted bytes, and must not be worse than the one-step arm.

These radial anchors are development evidence only. No full-episode or scene
transfer claim is authorized until the four-anchor gate passes.

## Development result

The persistent V96 arm improved the matched static full-payload baseline by
6.188% and 7.206% at the two M24 anchors, but only 1.566% and 4.955% at the two
X36 anchors.  The frozen cross-scale gate therefore passed at 2 of 4 anchors
and remains closed.  These gains support adaptive posterior-participation
control on M24; they do not yet support a general cross-scale claim or a claim
that dynamic physical routing itself is effective.  See
`ADDRESSABLE_RISK_ADAPTIVE_PAYLOAD_V96_FINDING.md` for the matched baseline
contract and the exact comparison.
