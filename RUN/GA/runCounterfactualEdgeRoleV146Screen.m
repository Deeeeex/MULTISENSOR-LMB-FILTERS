function [artifactPath, screen] = ...
        runCounterfactualEdgeRoleV146Screen(presetName, options)
% RUNCOUNTERFACTUALEDGEROLEV146SCREEN Evaluate one frozen headroom arm.

if nargin < 2 || isempty(options)
    options = struct();
end
[artifactPath, screen] = runFormationIsolatedDualPayloadV136Screen( ...
    presetName, options, getCounterfactualEdgeRoleV146Protocol());
end
