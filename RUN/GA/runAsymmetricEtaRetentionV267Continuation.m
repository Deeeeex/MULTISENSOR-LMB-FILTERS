function [reportPath, result] = ...
        runAsymmetricEtaRetentionV267Continuation(options)
% RUNASYMMETRICETARETENTIONV267CONTINUATION One support-safe arm.

if nargin < 1 || isempty(options), options = struct(); end
options.variant = 'v267';
[reportPath, result] = ...
    runLabelSelectiveRiskShortcutV265Continuation(options);
end
