function [artifactPath, screen] = ...
    runPostFusionLabelReadoutV138Screen(presetName, options)
% RUNPOSTFUSIONLABELREADOUTV138SCREEN Post-fusion label output gate.

if nargin < 2
    options = struct();
end
[artifactPath, screen] = ...
    runFormationIsolatedDualPayloadV136Screen( ...
        presetName, options, getPostFusionLabelReadoutV138Protocol());
end
