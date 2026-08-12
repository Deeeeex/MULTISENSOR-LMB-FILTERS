function context = buildIndependentLocalAnchorV128ExecutionContext( ...
        presetName, seed, currentTime, actionName, scheduleEnabled)
% BUILDINDEPENDENTLOCALANCHORV128EXECUTIONCONTEXT Frozen V128 context.

protocol = getIndependentLocalAnchorV128Protocol();
if ~ischar(presetName) || isempty(presetName) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || ...
        ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(scheduleEnabled) || ...
        ~(islogical(scheduleEnabled) || isnumeric(scheduleEnabled))
    error('IndependentLocalAnchorV128:InvalidExecutionContext', ...
        'The V128 execution-context input is invalid.');
end
context = struct();
context.contractVersion = 'independent-local-anchor-v128-context-v1';
context.capability = 'independent-local-anchor-v128-development';
context.action = 'filter-independent-local-anchor-v128-development';
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
