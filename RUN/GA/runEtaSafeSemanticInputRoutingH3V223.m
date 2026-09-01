function [reportPath, result] = ...
        runEtaSafeSemanticInputRoutingH3V223(options)
% RUNETASAFESEMANTICINPUTROUTINGH3V223 Frozen receiver-specific screens.

if nargin < 1 || isempty(options)
    options = struct();
end
options.etaProjectionEnabled = true;
[reportPath, result] = ...
    runSinglePassSemanticInputRoutingH3V221(options);
if ~strcmp(result.contractVersion, ...
        'eta-safe-semantic-input-routing-h3-v223-v1') || ...
        ~result.etaProjectionEnabled
    error('EtaSafeSemanticInputRoutingV223:WrapperDrift', ...
        'The V223 wrapper did not produce an eta-safe result.');
end
end
