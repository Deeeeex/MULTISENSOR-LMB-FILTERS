function [reportPath, result] = ...
        runFormationCoordinatedPosteriorRepairV185X36T72(options)
% RUNFORMATIONCOORDINATEDPOSTERIORREPAIRV185X36T72 Recursive V185 screen.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getFormationCoordinatedPosteriorRepairV185Protocol();
if exist(protocol.learnedModelPath, 'file') ~= 2 || ...
        exist(protocol.learnedPolicyPath, 'file') ~= 2
    error('FormationPosteriorRepairV185:MissingPolicy', ...
        'Freeze the V185 formation-coordinated policy first.');
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
