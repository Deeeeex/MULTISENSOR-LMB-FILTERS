function [reportPath, result] = ...
        runLocalPosteriorRollbackV127X36T72(options)
% RUNLOCALPOSTERIORROLLBACKV127X36T72 Paired local-state attribution.

if nargin < 1 || isempty(options)
    options = struct();
end
options.protocolOverride = getLocalPosteriorRollbackV127Protocol();
[reportPath, result] = runProtectionOnlyH8V105X36T72(options);
end
