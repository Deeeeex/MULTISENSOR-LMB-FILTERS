function context = ...
    buildSafeGraphOptionDwellV154ExecutionContext( ...
        presetName, seed, policyName)
% BUILDSAFEGRAPHOPTIONDWELLV154EXECUTIONCONTEXT Frozen pilot permit.

protocol = getSafeGraphOptionDwellV154Protocol();
presetIdx = find(strcmp(protocol.presetNames, presetName));
validModes = arrayfun(@(rankIndex) sprintf('%s%d', ...
    protocol.optionArmPrefix, rankIndex), ...
    protocol.optionRanks, 'UniformOutput', false);
if numel(presetIdx) ~= 1 || ...
        ~ismember(seed, protocol.openedDevelopmentSeeds) || ...
        ~ischar(policyName) || ~ismember(policyName, validModes)
    error('SafeGraphOptionDwellV154:InvalidExecutionContext', ...
        'The V154 execution request is outside the frozen pilot.');
end

context = struct();
context.contractVersion = ...
    'safe-graph-option-dwell-v154-execution-context-v1';
context.capability = 'safe-graph-option-dwell-v154-development';
context.action = 'filter-safe-graph-option-dwell-v154-development';
context.protocolId = protocol.id;
context.evidenceSplit = protocol.evidenceSplit;
context.phase = 'pilot-arm';
context.presetName = presetName;
context.seed = seed;
context.currentTime = protocol.anchorTimes(presetIdx);
context.horizonSteps = protocol.horizon;
context.measurementTimeCount = ...
    context.currentTime + context.horizonSteps - 1;
context.policyName = policyName;
context.trackingOutcomeScoringAuthorized = true;
context.developmentEvidenceOnly = true;
end
