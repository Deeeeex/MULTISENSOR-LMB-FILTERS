function context = buildTimeExpandedPathSelectiveV88ExecutionContext( ...
        presetName, seed, currentTime, actionName, routeEnabled)
% BUILDTIMEEXPANDEDPATHSELECTIVEV88EXECUTIONCONTEXT Opened primary cases.

protocol = getTimeExpandedPathSelectiveV88Protocol();
registered = false;
for caseInfo = protocol.cases
    registered = registered || ( ...
        strcmp(presetName, caseInfo.presetName) && ...
        seed == caseInfo.seed && currentTime == caseInfo.currentTime);
end
if ~registered || ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(routeEnabled) || ...
        ~(islogical(routeEnabled) || isnumeric(routeEnabled))
    error('TimeExpandedPathSelectiveV88:InvalidExecutionContext', ...
        'The V88 request is not a registered primary case.');
end
context = struct();
context.contractVersion = ...
    'time-expanded-path-selective-primary-h3-v88-context-v1';
context.capability = ...
    'time-expanded-path-selective-primary-h3-development';
context.action = ...
    'filter-time-expanded-path-selective-primary-h3-development';
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
