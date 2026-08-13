function [artifactPath, screen] = ...
    runCombinedOutputSafetyV139Screen(presetName, options)
% RUNCOMBINEDOUTPUTSAFETYV139SCREEN Complementary output-only gates.

if nargin < 2
    options = struct();
end
[artifactPath, screen] = ...
    runFormationIsolatedDualPayloadV136Screen( ...
        presetName, options, getCombinedOutputSafetyV139Protocol());
end
