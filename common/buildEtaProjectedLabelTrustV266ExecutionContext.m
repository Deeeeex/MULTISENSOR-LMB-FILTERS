function context = ...
        buildEtaProjectedLabelTrustV266ExecutionContext( ...
            presetName, seed, measurementTimeCount)
% BUILDETAPROJECTEDLABELTRUSTV266EXECUTIONCONTEXT Dev permit.

protocol = getEtaProjectedLabelTrustV266Protocol();
if ~ischar(presetName) || ...
        ~ismember(presetName, protocol.allowedPresets) || ...
        ~ismember(seed, protocol.allowedSeeds) || ...
        measurementTimeCount ~= protocol.continuationEndTime
    error('EtaProjectedLabelTrustV266:InvalidExecutionContextInput', ...
        'V266 requires its registered continuation.');
end
context = struct();
context.contractVersion = ...
    'eta-projected-label-trust-v266-context-v1';
context.capability = 'eta-projected-label-trust-v266-development';
context.action = 'filter-eta-projected-label-trust-v266-development';
context.protocolId = protocol.id;
context.presetName = presetName;
context.seed = seed;
context.armId = protocol.armId;
context.measurementTimeCount = measurementTimeCount;
context.continuationStartTime = protocol.continuationStartTime;
context.policyName = protocol.policyName;
context.developmentEvidenceOnly = true;
end
