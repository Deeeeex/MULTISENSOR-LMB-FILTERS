function [reportPath, result] = ...
        runOpenedSafeFormationV110X36T72(options)
% RUNOPENEDSAFEFORMATIONV110X36T72 Perfect formation-risk upper bound.

if nargin < 1 || isempty(options)
    options = struct();
end
options.protocolOverride = getOpenedSafeFormationV110Protocol();
[reportPath, result] = runProtectionReleaseOracleV106X36T72(options);
end
