function context = buildF6ReferenceRowCompositionV124ExecutionContext( ...
        presetName, seed, currentTime, actionName, scheduleEnabled)
% BUILDF6REFERENCEROWCOMPOSITIONV124EXECUTIONCONTEXT Frozen V124 context.

protocol = getF6ReferenceRowCompositionV124Protocol();
if ~ischar(presetName) || isempty(presetName) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || ...
        ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(scheduleEnabled) || ...
        ~(islogical(scheduleEnabled) || isnumeric(scheduleEnabled))
    error('F6ReferenceRowV124:InvalidExecutionContext', ...
        'The V124 execution-context input is invalid.');
end
context = struct();
context.contractVersion = 'f6-reference-row-composition-v124-context-v1';
context.capability = 'f6-reference-row-composition-v124-development';
context.action = 'filter-f6-reference-row-composition-v124-development';
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
