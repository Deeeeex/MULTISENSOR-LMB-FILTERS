function context = ...
        buildAsymmetricEtaRetentionV267ExecutionContext( ...
            presetName, seed, measurementTimeCount)
% BUILDASYMMETRICETARETENTIONV267EXECUTIONCONTEXT Dev permit.

protocol = getAsymmetricEtaRetentionV267Protocol();
if ~ischar(presetName) || ...
        ~ismember(presetName, protocol.allowedPresets) || ...
        ~ismember(seed, protocol.allowedSeeds) || ...
        measurementTimeCount ~= protocol.continuationEndTime
    error('AsymmetricEtaRetentionV267:InvalidExecutionContextInput', ...
        'V267 requires its registered continuation.');
end
context = struct();
context.contractVersion = ...
    'asymmetric-eta-retention-v267-context-v1';
context.capability = 'asymmetric-eta-retention-v267-development';
context.action = 'filter-asymmetric-eta-retention-v267-development';
context.protocolId = protocol.id;
context.presetName = presetName;
context.seed = seed;
context.armId = protocol.armId;
context.measurementTimeCount = measurementTimeCount;
context.continuationStartTime = protocol.continuationStartTime;
context.policyName = protocol.policyName;
context.developmentEvidenceOnly = true;
end
