function context = buildMixedCarrierInsertionV121ExecutionContext( ...
        presetName, seed, currentTime, actionName, scheduleEnabled)
% BUILDMIXEDCARRIERINSERTIONV121EXECUTIONCONTEXT Frozen V121 context.

protocol = getMixedCarrierInsertionV121Protocol();
if ~ischar(presetName) || isempty(presetName) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || ...
        ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(scheduleEnabled) || ...
        ~(islogical(scheduleEnabled) || isnumeric(scheduleEnabled))
    error('MixedCarrierInsertionV121:InvalidExecutionContext', ...
        'The V121 execution-context input is invalid.');
end
context = struct();
context.contractVersion = 'mixed-carrier-insertion-v121-context-v1';
context.capability = 'mixed-carrier-insertion-v121-development';
context.action = 'filter-mixed-carrier-insertion-v121-development';
context.protocolId = protocol.id;
context.phase = 'opened-return';
context.presetName = presetName;
context.seed = seed;
context.currentTime = currentTime;
context.measurementTimeCount = ...
    currentTime + protocol.horizonSteps - 1;
context.policyName = protocol.outcomePolicyName;
context.actionName = actionName;
context.scheduleEnabled = logical(scheduleEnabled);
context.onlineReselectionEnabled = false;
context.developmentEvidenceOnly = true;
end
