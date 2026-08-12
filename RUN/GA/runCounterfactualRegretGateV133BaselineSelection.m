function [selectionPath, selection] = ...
    runCounterfactualRegretGateV133BaselineSelection(options)
% RUNCOUNTERFACTUALREGRETGATEV133BASELINESELECTION Freeze static carriers.
%
% Every preset x seed x carrier arm is checkpointed independently before
% the paired complete-trajectory records are aggregated.  Re-running this
% entry point therefore resumes completed arms instead of discarding a
% partially completed long experiment.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getCounterfactualRegretGateV133Protocol();
for caseInfo = protocol.scaleCases
    for seed = protocol.developmentSeeds
        for carrierIdx = 1:numel(protocol.referenceCarrierModes)
            runCounterfactualRegretGateV133BaselineShard( ...
                caseInfo.presetName, seed, ...
                protocol.referenceCarrierModes{carrierIdx}, options);
        end
    end
end
[selectionPath, selection] = ...
    finalizeCounterfactualRegretGateV133BaselineSelection(options);
end
