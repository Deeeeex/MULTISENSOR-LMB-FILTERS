function [reportPath, result] = ...
        runPositiveValueReferenceLabelV157X36T72(options)
% RUNPOSITIVEVALUEREFERENCELABELV157X36T72 Truth-valued label headroom.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getPositiveValueReferenceLabelV157Protocol();
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
