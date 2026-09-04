function context = buildMinimalRiskCycleV264ExecutionContext( ...
        presetName, seed, measurementTimeCount)
% BUILDMINIMALRISKCYCLEV264EXECUTIONCONTEXT Dev permit.

protocol = getMinimalRiskCycleV264Protocol();
if ~ischar(presetName) || ...
        ~ismember(presetName, protocol.allowedPresets) || ...
        ~ismember(seed, protocol.allowedSeeds) || ...
        measurementTimeCount ~= protocol.continuationEndTime
    error('MinimalRiskCycleV264:InvalidExecutionContextInput', ...
        'V264 requires its registered continuation.');
end
context = struct();
context.contractVersion = 'minimal-risk-cycle-v264-context-v1';
context.capability = 'minimal-risk-cycle-v264-development';
context.action = 'filter-minimal-risk-cycle-v264-development';
context.protocolId = protocol.id;
context.presetName = presetName;
context.seed = seed;
context.armId = protocol.armId;
context.measurementTimeCount = measurementTimeCount;
context.continuationStartTime = protocol.continuationStartTime;
context.policyName = protocol.policyName;
context.developmentEvidenceOnly = true;
end
