function [reportPath, result] = ...
        runExistenceMarginBeneficiaryV269Continuation(options)
% RUNEXISTENCEMARGINBENEFICIARYV269CONTINUATION One receiver-selection arm.

if nargin < 1 || isempty(options), options = struct(); end
options.variant = 'v269';
[reportPath, result] = ...
    runLabelSelectiveRiskShortcutV265Continuation(options);
end
