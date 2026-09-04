function [reportPath, result] = ...
        runEtaProjectedLabelTrustV266Continuation(options)
% RUNETAPROJECTEDLABELTRUSTV266CONTINUATION One projected-trust arm.

if nargin < 1 || isempty(options), options = struct(); end
options.variant = 'v266';
[reportPath, result] = ...
    runLabelSelectiveRiskShortcutV265Continuation(options);
end
