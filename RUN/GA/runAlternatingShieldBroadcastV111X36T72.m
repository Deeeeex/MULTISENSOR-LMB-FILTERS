function [reportPath, result] = ...
        runAlternatingShieldBroadcastV111X36T72(options)
% RUNALTERNATINGSHIELDBROADCASTV111X36T72 H=8 propagation headroom.

if nargin < 1 || isempty(options)
    options = struct();
end
options.protocolOverride = getAlternatingShieldBroadcastV111Protocol();
[reportPath, result] = runProtectionReleaseOracleV106X36T72(options);
end
