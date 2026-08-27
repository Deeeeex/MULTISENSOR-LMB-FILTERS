function [artifactPath, screen] = ...
        runReferenceCardinalityReadoutV140Screen(presetName, options)
% RUNREFERENCECARDINALITYREADOUTV140SCREEN Conditional-MAP output projection.

if nargin < 2
    options = struct();
end
[artifactPath, screen] = ...
    runFormationIsolatedDualPayloadV136Screen( ...
        presetName, options, getReferenceCardinalityReadoutV140Protocol());
end
