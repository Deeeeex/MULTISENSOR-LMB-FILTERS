function context = buildBraidedHandoverH3V84ExecutionContext( ...
        presetName, seed, currentTime, actionName, routeEnabled)
% BUILDBRAIDEDHANDOVERH3V84EXECUTIONCONTEXT Primary opened cases.

protocol = getBraidedHandoverH3V84Protocol();
registered = false;
for caseInfo = protocol.cases
    registered = registered || ( ...
        strcmp(presetName, caseInfo.presetName) && ...
        seed == caseInfo.seed && currentTime == caseInfo.currentTime);
end
if ~registered || ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(routeEnabled) || ...
        ~(islogical(routeEnabled) || isnumeric(routeEnabled))
    error('BraidedHandoverH3V84:InvalidExecutionContext', ...
        'The V84 H=3 request is not a registered primary case.');
end
context = struct();
context.contractVersion = ...
    'braided-handover-primary-h3-v84-context-v1';
context.capability = 'braided-handover-primary-h3-development';
context.action = 'filter-braided-handover-primary-h3-development';
context.protocolId = protocol.id;
context.phase = 'opened-return';
context.presetName = presetName;
context.seed = seed;
context.currentTime = currentTime;
context.measurementTimeCount = ...
    currentTime + protocol.horizonSteps - 1;
context.policyName = protocol.outcomePolicyName;
context.actionName = actionName;
context.routeEnabled = logical(routeEnabled);
context.developmentEvidenceOnly = true;
end
