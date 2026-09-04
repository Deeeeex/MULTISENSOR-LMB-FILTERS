function context = ...
        buildRiskEpisodeLatchedFormationShortcutV262ExecutionContext( ...
            presetName, seed, measurementTimeCount)
% BUILDRISKEPISODELATCHEDFORMATIONSHORTCUTV262EXECUTIONCONTEXT Dev permit.

protocol = getRiskEpisodeLatchedFormationShortcutV262Protocol();
if ~ischar(presetName) || ...
        ~ismember(presetName, protocol.allowedPresets) || ...
        ~ismember(seed, protocol.allowedSeeds) || ...
        measurementTimeCount ~= protocol.continuationEndTime
    error('RiskEpisodeLatchV262:InvalidExecutionContextInput', ...
        'V262 requires its registered continuation.');
end
context = struct();
context.contractVersion = ...
    'risk-episode-latched-formation-shortcut-v262-context-v1';
context.capability = ...
    'risk-episode-latched-formation-shortcut-v262-development';
context.action = ...
    'filter-risk-episode-latched-formation-shortcut-v262-development';
context.protocolId = protocol.id;
context.presetName = presetName;
context.seed = seed;
context.armId = protocol.armId;
context.measurementTimeCount = measurementTimeCount;
context.continuationStartTime = protocol.continuationStartTime;
context.policyName = protocol.policyName;
context.developmentEvidenceOnly = true;
end

