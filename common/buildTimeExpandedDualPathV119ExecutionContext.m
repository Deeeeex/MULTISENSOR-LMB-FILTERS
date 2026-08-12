function context = buildTimeExpandedDualPathV119ExecutionContext( ...
        presetName, seed, currentTime, actionName, scheduleEnabled)
% BUILDTIMEEXPANDEDDUALPATHV119EXECUTIONCONTEXT Frozen V119 context.

protocol = getTimeExpandedDualPathV119Protocol();
if ~ischar(presetName) || isempty(presetName) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || ...
        ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(scheduleEnabled) || ...
        ~(islogical(scheduleEnabled) || isnumeric(scheduleEnabled))
    error('TimeExpandedDualPathV119:InvalidExecutionContext', ...
        'The V119 execution-context input is invalid.');
end
context = struct();
context.contractVersion = 'time-expanded-dual-path-v119-context-v1';
context.capability = 'time-expanded-dual-path-v119-development';
context.action = 'filter-time-expanded-dual-path-v119-development';
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
