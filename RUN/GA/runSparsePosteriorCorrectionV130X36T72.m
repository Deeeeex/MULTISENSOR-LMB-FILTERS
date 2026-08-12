function [reportPath, result] = ...
        runSparsePosteriorCorrectionV130X36T72(options)
% RUNSPARSEPOSTERIORCORRECTIONV130X36T72 Paired hybrid attribution.

if nargin < 1 || isempty(options)
    options = struct();
end
options.protocolOverride = getSparsePosteriorCorrectionV130Protocol();
[reportPath, result] = runProtectionOnlyH8V105X36T72(options);
end
