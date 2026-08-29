function context = ...
    buildSafeGraphCodebookOracleV152ExecutionContext( ...
        presetName, seed, phase, policyName)
% BUILDSAFEGRAPHCODEBOOKORACLEV152EXECUTIONCONTEXT Frozen pilot permit.

protocol = getSafeGraphCodebookOracleV152Protocol();
presetIdx = find(strcmp(protocol.presetNames, presetName));
phase = lower(strrep(char(phase), '_', '-'));
if numel(presetIdx) ~= 1 || ...
        ~ismember(seed, protocol.openedDevelopmentSeeds) || ...
        ~ismember(phase, {'reference-cache', 'pilot-arm'}) || ...
        ~ischar(policyName) || isempty(policyName)
    error('SafeGraphCodebookV152:InvalidExecutionContext', ...
        'The V152 execution request is outside the frozen pilot.');
end
if strcmp(phase, 'reference-cache')
    if ~strcmp(policyName, 'static-full-posterior-reference-cache')
        error('SafeGraphCodebookV152:InvalidExecutionContext', ...
            'The V152 cache phase requires its static reference policy.');
    end
    measurementTimeCount = protocol.anchorTimes(presetIdx);
    trackingOutcomeScoringAuthorized = false;
else
    validPilotModes = [protocol.staticArmModes, arrayfun( ...
        @(rankIndex) sprintf('%s%d', ...
            protocol.proposalArmPrefix, rankIndex), ...
        protocol.proposalRanks, 'UniformOutput', false)];
    if ~ismember(policyName, validPilotModes)
        error('SafeGraphCodebookV152:InvalidExecutionContext', ...
            'The V152 pilot policy is outside the frozen codebook.');
    end
    measurementTimeCount = protocol.anchorTimes(presetIdx) + ...
        protocol.horizon - 1;
    trackingOutcomeScoringAuthorized = true;
end

context = struct();
context.contractVersion = ...
    'safe-graph-codebook-oracle-v152-execution-context-v1';
context.capability = 'safe-graph-codebook-oracle-v152-development';
context.action = 'filter-safe-graph-codebook-oracle-v152-development';
context.protocolId = protocol.id;
context.evidenceSplit = protocol.evidenceSplit;
context.phase = phase;
context.presetName = presetName;
context.seed = seed;
context.currentTime = protocol.anchorTimes(presetIdx);
context.horizonSteps = protocol.horizon;
context.measurementTimeCount = measurementTimeCount;
context.policyName = policyName;
context.trackingOutcomeScoringAuthorized = ...
    trackingOutcomeScoringAuthorized;
context.developmentEvidenceOnly = true;
end
