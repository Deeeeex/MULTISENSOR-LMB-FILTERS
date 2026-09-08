# Empty-array shape repair before completed preflight trajectories

The first native MATLAB preflight exited with code 1 at the opportunity/pD
equality assertion on the first empty local frame. No result file or complete
arm existed. The empty object field concatenation produces a 0-by-0 array,
whereas zeros(1,numel(predicted)) produces a 1-by-0 array; isequal checks shape
as well as contents. This also affects the shape assertion in the new negative
support helper, not the tracking model or evidence rule.

Before the equality check, reshape the already computed pD vector to the
opportunity array's size. Values, dimensions for nonempty tracks and all
probability calculations are unchanged. The previous generated runner,
generator, freeze script and original 1330-file source manifest are preserved
under initial_preflight/. Its manifest records paths as they stood during the
failed attempt; the three replaced file contents can be checked against the
same basenames in that archive. Other paths remain unchanged.

The failure log is retained as preflight_initial_shape_failure.log. Create a
new source manifest including these originals and this amendment before any
retry. The primary, ablations, cohorts and decision rule are unchanged. This
repair precedes any completed preflight trajectory or outcome inspection.
