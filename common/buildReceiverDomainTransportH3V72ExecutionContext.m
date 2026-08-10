function context = buildReceiverDomainTransportH3V72ExecutionContext( ...
        presetName, seed, currentTime, actionName, routeEnabled)
% BUILDRECEIVERDOMAINTRANSPORTH3V72EXECUTIONCONTEXT Exact opened cases.

protocol = getReceiverDomainTransportH3V72Protocol();
registered = false;
for caseInfo = protocol.cases
    registered = registered || ( ...
        strcmp(presetName, caseInfo.presetName) && ...
        seed == caseInfo.seed && currentTime == caseInfo.currentTime);
end
if ~registered || ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(routeEnabled) || ...
        ~(islogical(routeEnabled) || isnumeric(routeEnabled))
    error('ReceiverDomainTransportV72:InvalidExecutionContext', ...
        'The V72 execution context is not a registered opened case.');
end

context = struct();
context.contractVersion = ...
    'receiver-domain-transport-h3-v72-context-v1';
context.capability = 'receiver-domain-transport-h3-development';
context.action = 'filter-receiver-domain-transport-h3-development';
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
