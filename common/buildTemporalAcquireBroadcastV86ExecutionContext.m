function context = buildTemporalAcquireBroadcastV86ExecutionContext( ...
        presetName, seed, currentTime, actionName, routeEnabled)
% BUILDTEMPORALACQUIREBROADCASTV86EXECUTIONCONTEXT Opened primary cases.

protocol = getTemporalAcquireBroadcastV86Protocol();
registered = false;
for caseInfo = protocol.cases
    registered = registered || ( ...
        strcmp(presetName, caseInfo.presetName) && ...
        seed == caseInfo.seed && currentTime == caseInfo.currentTime);
end
if ~registered || ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(routeEnabled) || ...
        ~(islogical(routeEnabled) || isnumeric(routeEnabled))
    error('TemporalAcquireBroadcastV86:InvalidExecutionContext', ...
        'The V86 request is not a registered primary case.');
end
context = struct();
context.contractVersion = ...
    'temporal-acquire-broadcast-primary-h3-v86-context-v1';
context.capability = ...
    'temporal-acquire-broadcast-primary-h3-development';
context.action = ...
    'filter-temporal-acquire-broadcast-primary-h3-development';
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
