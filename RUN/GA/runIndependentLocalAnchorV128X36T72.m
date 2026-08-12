function [reportPath, result] = ...
        runIndependentLocalAnchorV128X36T72(options)
% RUNINDEPENDENTLOCALANCHORV128X36T72 Paired local-anchor attribution.

if nargin < 1 || isempty(options)
    options = struct();
end
options.protocolOverride = getIndependentLocalAnchorV128Protocol();
[reportPath, result] = runProtectionOnlyH8V105X36T72(options);
end
