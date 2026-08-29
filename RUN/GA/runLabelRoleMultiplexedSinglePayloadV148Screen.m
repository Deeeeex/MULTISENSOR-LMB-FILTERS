function [artifactPath, screen] = ...
        runLabelRoleMultiplexedSinglePayloadV148Screen(presetName, options)
% RUNLABELROLEMULTIPLEXEDSINGLEPAYLOADV148SCREEN Evaluate label-wise W/R.

if nargin < 2 || isempty(options)
    options = struct();
end
[artifactPath, screen] = runFormationIsolatedDualPayloadV136Screen( ...
    presetName, options, ...
    getLabelRoleMultiplexedSinglePayloadV148Protocol());
end
