function [reportPath, result] = ...
        runSparseReferenceLabelRollbackV156X36T72( ...
            maximumLabelEdits, options)
% RUNSPARSEREFERENCELABELROLLBACKV156X36T72 One frozen V156 capacity arm.

if nargin < 1 || isempty(maximumLabelEdits)
    maximumLabelEdits = 2;
end
if nargin < 2 || isempty(options)
    options = struct();
end
protocol = ...
    getSparseReferenceLabelRollbackV156Protocol(maximumLabelEdits);
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
