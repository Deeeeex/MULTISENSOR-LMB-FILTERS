function [reportPath, result] = ...
        runIntermittentLightNetworkAnchorV131X36T72(options)
% RUNINTERMITTENTLIGHTNETWORKANCHORV131X36T72 Paired anchor attribution.

if nargin < 1 || isempty(options)
    options = struct();
end
options.protocolOverride = ...
    getIntermittentLightNetworkAnchorV131Protocol();
[reportPath, result] = runProtectionOnlyH8V105X36T72(options);
end
