function [reportPath, result] = ...
        runRolloutAwareOneHopSafeLabelV179X36T72(options)
% RUNROLLOUTAWAREONEHOPSAFELABELV179X36T72 Recursive V179 screen.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getRolloutAwareOneHopSafeLabelV179Protocol();
if exist(protocol.learnedModelPath, 'file') ~= 2 || ...
        exist(protocol.learnedPolicyPath, 'file') ~= 2
    error('RolloutAwareOneHopSafeLabelV179:MissingPolicy', ...
        'Freeze the V178 compact rollout-aware policy first.');
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
