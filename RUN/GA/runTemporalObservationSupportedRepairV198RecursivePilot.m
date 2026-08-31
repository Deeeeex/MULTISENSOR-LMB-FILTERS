function [reportPath, result] = ...
        runTemporalObservationSupportedRepairV198RecursivePilot( ...
            scaleName, options)
% RUNTEMPORALOBSERVATIONSUPPORTEDREPAIRV198RECURSIVEPILOT Paired run.

if nargin < 2 || isempty(options)
    options = struct();
end
horizonSteps = getField(options, 'horizonSteps', 3);
options.horizonSteps = horizonSteps;
options.protocolOverride = ...
    getTemporalObservationSupportedRepairV198Protocol(horizonSteps);
options.versionTag = 'V198';
[reportPath, result] = ...
    runBudgetedObservationSupportedRepairV197RecursivePilot( ...
        scaleName, options);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
