function context = ...
        buildFormationSemanticContinuityShieldV236ExecutionContext( ...
            measurementTimeCount)
% BUILDFORMATIONSEMANTICCONTINUITYSHIELDV236EXECUTIONCONTEXT Teacher run.

protocol = getFormationSemanticContinuityShieldV236Protocol();
if ~isscalar(measurementTimeCount) || ...
        ~isfinite(measurementTimeCount) || ...
        measurementTimeCount < protocol.shieldStartTime || ...
        measurementTimeCount ~= round(measurementTimeCount)
    error('FormationSemanticContinuityV236:InvalidExecutionContextInput', ...
        'V236 requires a full time axis containing its teacher switch.');
end
context = struct();
context.contractVersion = ...
    'formation-semantic-continuity-shield-v236-context-v1';
context.capability = ...
    'formation-semantic-continuity-shield-v236-development';
context.action = ...
    'filter-formation-semantic-continuity-shield-v236-development';
context.protocolId = protocol.id;
context.presetName = protocol.presetName;
context.seed = protocol.seed;
context.splitName = protocol.splitName;
context.armId = protocol.armId;
context.measurementTimeCount = measurementTimeCount;
context.policyName = protocol.policyName;
context.targetFormationId = protocol.targetFormationId;
context.shieldStartTime = protocol.shieldStartTime;
context.scheduleSelectedFromOfflineOutcome = true;
context.developmentEvidenceOnly = true;
end
