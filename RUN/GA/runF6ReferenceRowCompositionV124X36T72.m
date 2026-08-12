function [reportPath, result] = ...
        runF6ReferenceRowCompositionV124X36T72(options)
% RUNF6REFERENCEROWCOMPOSITIONV124X36T72 Paired V124 ablation.

if nargin < 1 || isempty(options)
    options = struct();
end
options.protocolOverride = getF6ReferenceRowCompositionV124Protocol();
[reportPath, result] = ...
    runProjectedFormationRowCompositionV123X36T72(options);
end
