function [artifactPath, screen] = ...
        runProtectionLoadGatedRoleV145Screen(presetName, options)
% RUNPROTECTIONLOADGATEDROLEV145SCREEN Frozen observable role gate.

if nargin < 2
    options = struct();
end
[artifactPath, screen] = ...
    runFormationIsolatedDualPayloadV136Screen( ...
        presetName, options, getProtectionLoadGatedRoleV145Protocol());
end

