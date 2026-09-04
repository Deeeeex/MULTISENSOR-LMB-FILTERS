function context = ...
        buildExistenceMarginBeneficiaryV269ExecutionContext( ...
            presetName, seed, measurementTimeCount)
% BUILDEXISTENCEMARGINBENEFICIARYV269EXECUTIONCONTEXT Dev permit.

protocol = getExistenceMarginBeneficiaryV269Protocol();
if ~ischar(presetName) || ...
        ~ismember(presetName, protocol.allowedPresets) || ...
        ~ismember(seed, protocol.allowedSeeds) || ...
        measurementTimeCount ~= protocol.continuationEndTime
    error('ExistenceMarginBeneficiaryV269:InvalidContextInput', ...
        'V269 requires its registered continuation.');
end
context = struct();
context.contractVersion = ...
    'existence-margin-beneficiary-v269-context-v1';
context.capability = 'existence-margin-beneficiary-v269-development';
context.action = 'filter-existence-margin-beneficiary-v269-development';
context.protocolId = protocol.id;
context.presetName = presetName;
context.seed = seed;
context.armId = protocol.armId;
context.measurementTimeCount = measurementTimeCount;
context.continuationStartTime = protocol.continuationStartTime;
context.policyName = protocol.policyName;
context.developmentEvidenceOnly = true;
end
