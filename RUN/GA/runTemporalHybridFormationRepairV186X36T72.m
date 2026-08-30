function [reportPath, result] = ...
        runTemporalHybridFormationRepairV186X36T72(options)
% RUNTEMPORALHYBRIDFORMATIONREPAIRV186X36T72 Recursive V186 screen.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getTemporalHybridFormationRepairV186Protocol();
requiredPaths = {protocol.learnedModelPath, protocol.learnedPolicyPath, ...
    protocol.formationCoordinatedPosteriorRepairModelPath, ...
    protocol.formationCoordinatedPosteriorRepairPolicyPath, ...
    protocol.preflightResultPath};
if any(cellfun(@(path) exist(path, 'file') ~= 2, requiredPaths))
    error('TemporalHybridFormationRepairV186:MissingPolicy', ...
        'The frozen V179 and V185 policy artifacts are required.');
end
loadedPreflight = load(protocol.preflightResultPath, 'preflight');
if ~isfield(loadedPreflight, 'preflight') || ...
        ~loadedPreflight.preflight.gatePassed
    error('TemporalHybridFormationRepairV186:PreflightRejected', ...
        'The source-local Top-K preflight rejected recursive V186.');
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
