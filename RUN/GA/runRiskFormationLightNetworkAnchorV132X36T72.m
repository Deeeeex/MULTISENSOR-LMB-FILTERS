function [reportPath, result] = ...
        runRiskFormationLightNetworkAnchorV132X36T72(options)
% RUNRISKFORMATIONLIGHTNETWORKANCHORV132X36T72 Paired reallocation test.

if nargin < 1 || isempty(options)
    options = struct();
end
options.protocolOverride = ...
    getRiskFormationLightNetworkAnchorV132Protocol();
[reportPath, result] = runProtectionOnlyH8V105X36T72(options);
end
