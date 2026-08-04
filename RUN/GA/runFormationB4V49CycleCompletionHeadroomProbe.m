function result = runFormationB4V49CycleCompletionHeadroomProbe(options)
% RUNFORMATIONB4V49CYCLECOMPLETIONHEADROOMPROBE Route-only public entrypoint.

if nargin < 1
    options = struct();
end
if isfield(options, 'includeDeferAblation') && ...
        options.includeDeferAblation
    error('FormationB4V49CycleHeadroom:DeferNotPrimary', ...
        'Use the legacy three-arm runner for the defer ablation.');
end
options.includeDeferAblation = false;
result = runFormationB4V49RingSafeDeferHeadroomProbe(options);
end
