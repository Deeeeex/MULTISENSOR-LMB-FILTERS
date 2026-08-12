function [reportPath, result] = ...
        runShadowStateRollbackV126X36T72(options)
% RUNSHADOWSTATEROLLBACKV126X36T72 Paired V126 recovery upper bound.

if nargin < 1 || isempty(options)
    options = struct();
end
options.protocolOverride = getShadowStateRollbackV126Protocol();
[reportPath, result] = runProtectionOnlyH8V105X36T72(options);
end
