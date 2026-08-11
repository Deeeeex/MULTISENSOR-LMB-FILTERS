function context = buildAlternativeGatewayV117ExecutionContext( ...
        presetName, seed, currentTime, actionName, scheduleEnabled)
% BUILDALTERNATIVEGATEWAYV117EXECUTIONCONTEXT Frozen V117 context.

protocol = getAlternativeGatewayV117Protocol();
if ~ischar(presetName) || isempty(presetName) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || ...
        ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(scheduleEnabled) || ...
        ~(islogical(scheduleEnabled) || isnumeric(scheduleEnabled))
    error('AlternativeGatewayV117:InvalidExecutionContext', ...
        'The V117 execution-context input is invalid.');
end
context = struct();
context.contractVersion = 'alternative-gateway-v117-context-v1';
context.capability = 'alternative-gateway-v117-development';
context.action = 'filter-alternative-gateway-v117-development';
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
