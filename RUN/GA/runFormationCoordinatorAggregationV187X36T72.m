function [reportPath, result] = ...
        runFormationCoordinatorAggregationV187X36T72(options)
% RUNFORMATIONCOORDINATORAGGREGATIONV187X36T72 Recursive V187 screen.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getFormationCoordinatorAggregationV187Protocol();
requiredPaths = {protocol.learnedModelPath, protocol.learnedPolicyPath, ...
    protocol.formationCoordinatedPosteriorRepairModelPath, ...
    protocol.formationCoordinatedPosteriorRepairPolicyPath, ...
    protocol.preflightResultPath};
if any(cellfun(@(path) exist(path, 'file') ~= 2, requiredPaths))
    error('FormationCoordinatorV187:MissingPolicy', ...
        'The frozen V179/V185 policies and V187 preflight are required.');
end
loadedPreflight = load(protocol.preflightResultPath, 'preflight');
if ~isfield(loadedPreflight, 'preflight') || ...
        ~loadedPreflight.preflight.gatePassed
    error('FormationCoordinatorV187:PreflightRejected', ...
        'The formation-coordinator preflight did not pass.');
end
options.protocolOverride = protocol;
options.outputRoot = getField(options, 'outputRoot', ...
    fullfile(protocol.headroomOutputRoot, 'x36_t72_h8'));
[reportPath, result] = runProtectionOnlyH8V105X36T72(options);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
