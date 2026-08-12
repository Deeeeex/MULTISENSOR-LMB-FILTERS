function [reportPath, result] = ...
        runOutwardReferenceCarrierV125X36T72(options)
% RUNOUTWARDREFERENCECARRIERV125X36T72 Paired V125 causal upper bound.

if nargin < 1 || isempty(options)
    options = struct();
end
options.protocolOverride = getOutwardReferenceCarrierV125Protocol();
[reportPath, result] = ...
    runProjectedFormationRowCompositionV123X36T72(options);
end
