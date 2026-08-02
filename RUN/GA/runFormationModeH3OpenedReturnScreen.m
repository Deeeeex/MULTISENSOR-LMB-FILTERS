function [reportPath, screen] = ...
    runFormationModeH3OpenedReturnScreen( ...
        presetName, seed, currentTime, options)
% RUNFORMATIONMODEH3OPENEDRETURNSCREEN Compatibility wrapper for H=3.
%
% Historical H=3 callers retain their registered contract.  Longer opened
% return horizons must call runFormationModeOpenedReturnScreen explicitly.

if nargin < 4 || isempty(options)
    options = struct();
end
if isfield(options, 'horizonSteps') && ...
        (~isscalar(options.horizonSteps) || ...
         ~isfinite(options.horizonSteps) || ...
         options.horizonSteps ~= 3)
    error('The H=3 compatibility wrapper only accepts horizonSteps=3.');
end
options.horizonSteps = 3;
[reportPath, screen] = runFormationModeOpenedReturnScreen( ...
    presetName, seed, currentTime, options);
end
