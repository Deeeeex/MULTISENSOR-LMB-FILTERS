function context = buildSparsePosteriorCorrectionV130ExecutionContext( ...
        presetName, seed, currentTime, actionName, scheduleEnabled)
% BUILDSPARSEPOSTERIORCORRECTIONV130EXECUTIONCONTEXT Frozen V130 context.

protocol = getSparsePosteriorCorrectionV130Protocol();
if ~ischar(presetName) || isempty(presetName) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || ...
        ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(scheduleEnabled) || ...
        ~(islogical(scheduleEnabled) || isnumeric(scheduleEnabled))
    error('SparsePosteriorCorrectionV130:InvalidExecutionContext', ...
        'The V130 execution-context input is invalid.');
end
context = struct();
context.contractVersion = 'sparse-posterior-correction-v130-context-v1';
context.capability = 'sparse-posterior-correction-v130-development';
context.action = 'filter-sparse-posterior-correction-v130-development';
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
