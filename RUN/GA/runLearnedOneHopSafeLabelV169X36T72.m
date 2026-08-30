function [reportPath, result] = ...
        runLearnedOneHopSafeLabelV169X36T72(options)
% RUNLEARNEDONEHOPSAFELABELV169X36T72 Recursive learned repair screen.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getLearnedOneHopSafeLabelV169Protocol();
if exist(protocol.learnedModelPath, 'file') ~= 2 || ...
        exist(protocol.learnedPolicyPath, 'file') ~= 2
    error('LearnedOneHopSafeLabelV169:MissingPolicy', ...
        ['Run the V170 compact-model, V174 conditional-model and V175 ', ...
         'policy preflight before the recursive V169 screen.']);
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
