function [reportPath, result] = ...
        runAttenuatedLightPayloadV129X36T72(options)
% RUNATTENUATEDLIGHTPAYLOADV129X36T72 Paired soft-protection attribution.

if nargin < 1 || isempty(options)
    options = struct();
end
options.protocolOverride = getAttenuatedLightPayloadV129Protocol();
[reportPath, result] = runProtectionOnlyH8V105X36T72(options);
end
