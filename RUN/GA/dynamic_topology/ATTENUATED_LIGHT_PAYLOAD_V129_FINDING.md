# V129 finding: uniform light input restores safety but spends the headroom

## Registered result

| Metric | V129 attenuated light payload | Strict gate |
|:--|--:|:--:|
| Mean E-OSPA gain vs static | +1.614% | fail |
| Minimum mature-page gain | +0.141% | fail |
| Minimum formation gain | -0.559% | fail |
| Minimum formation-time gain | -13.826% | fail |
| F6 peer terminal gain | +0.023% | pass |
| Worst-sensor gain | +6.982% | pass |
| Window / terminal consensus gain | +2.878% / +6.404% | pass |
| Fully accounted attempted-byte saving | +7.865% | pass |

V129 replaces every V105 control-only protected edge with a one-Gaussian
per-label posterior and applies a pre-registered 0.5 light-input reliability
factor.  No auxiliary state is maintained and the compressed messages are
included directly in the byte ledger.

## Mechanism conclusion

Uniform light input moves the two original V105 tail failures in the intended
direction.  F1's formation-average gain changes from -0.931% to +0.549%, and
F6 changes from -0.021% to +0.262%; F6 peers are nonnegative at the terminal
page.  The light posterior therefore contains useful coarse network
information that the control-only action removes.

The same action is too broad.  It reduces the V105 mean gain from +5.259% to
+1.614% and leaves a large F2 page-five regression (-13.826%).  This is not a
payload-size failure: V129 saves more attempted bytes than V105 because the
light payload replaces the relatively large control synopsis.  It is an
estimation-action allocation failure.  Network information should be added
only where protection debt emerges, rather than on every protected edge from
the first page.

V129 fails the registered gate and is retained only as a repository experiment
record.  It is not promoted to the canonical progress document.

## Method decision

The next candidate should retain V105 control-only protection as the default
and use the light posterior as a sparse safety correction.  The V126 mechanism
mask identifies an upper-bound action support (F2 at page five, F1 at pages
six through eight, and F6 at pages seven through eight).  A first attribution
should apply light payload only on those formation-page cells.  If this sparse
action preserves at least 5% mean gain while improving the local gate, the
remaining task is to replace that opened mask with a causal observable debt
trigger; if it does not, the light payload is not strong enough and further
weight sweeps are not justified.
