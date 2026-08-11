function [reportPath, result] = ...
        runExplicitLabelAbstentionV109X36T72(options)
% RUNEXPLICITLABELABSTENTIONV109X36T72 Neutral missing-label attribution.

if nargin < 1 || isempty(options)
    options = struct();
end
options.protocolOverride = getExplicitLabelAbstentionV109Protocol();
[reportPath, result] = runProtectionReleaseOracleV106X36T72(options);
end
