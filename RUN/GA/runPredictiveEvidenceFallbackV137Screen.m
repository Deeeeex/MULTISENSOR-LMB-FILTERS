function [artifactPath, screen] = ...
    runPredictiveEvidenceFallbackV137Screen(presetName, options)
% RUNPREDICTIVEEVIDENCEFALLBACKV137SCREEN Causal local W/R gate.

if nargin < 2
    options = struct();
end
[artifactPath, screen] = ...
    runFormationIsolatedDualPayloadV136Screen( ...
        presetName, options, getPredictiveEvidenceFallbackV137Protocol());
end
