function [reportPath, result] = ...
        runProtectionEarlyReleaseOracleV107X36T72(options)
% RUNPROTECTIONEARLYRELEASEORACLEV107X36T72 One-page-early oracle.

if nargin < 1 || isempty(options)
    options = struct();
end
options.protocolOverride = ...
    getProtectionEarlyReleaseOracleV107Protocol();
[reportPath, result] = ...
    runProtectionReleaseOracleV106X36T72(options);
end
