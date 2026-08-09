function context = ...
    buildTrackingAlignedX36ScheduleHeadroomV63ExecutionContext( ...
        phase, currentTime, actionName, scheduleEnabled)
% BUILDTRACKINGALIGNEDX36SCHEDULEHEADROOMV63EXECUTIONCONTEXT Two phases.

protocol = getTrackingAlignedX36ScheduleHeadroomV63Protocol();
if nargin < 4 || isempty(scheduleEnabled)
    scheduleEnabled = false;
end
if ~ischar(phase) || ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || ...
        ~isscalar(scheduleEnabled) || ...
        ~(islogical(scheduleEnabled) || isnumeric(scheduleEnabled))
    error('TrackingAlignedV63:InvalidExecutionContextInput', ...
        'The V63 phase, time, action name, or schedule flag is invalid.');
end
switch phase
    case 'reference-cache'
        if currentTime ~= max(protocol.anchorTimes) || scheduleEnabled
            error('TrackingAlignedV63:InvalidReferenceCacheContext', ...
                'The V63 cache phase must end at the last anchor.');
        end
        measurementTimeCount = currentTime;
        policyName = protocol.referencePolicyName;
    case 'opened-return'
        if ~ismember(currentTime, protocol.anchorTimes)
            error('TrackingAlignedV63:InvalidOpenedReturnContext', ...
                'The V63 opened-return time is not registered.');
        end
        measurementTimeCount = currentTime + protocol.horizonSteps - 1;
        policyName = protocol.outcomePolicyName;
    otherwise
        error('TrackingAlignedV63:UnknownPhase', ...
            'Unknown V63 execution phase.');
end

context = struct();
context.contractVersion = ...
    'tracking-aligned-x36-schedule-headroom-v63-context-v1';
context.capability = ...
    'tracking-aligned-x36-schedule-headroom-development';
context.action = ...
    'filter-tracking-aligned-x36-schedule-headroom-development';
context.protocolId = protocol.id;
context.phase = phase;
context.presetName = protocol.presetName;
context.seed = protocol.seed;
context.currentTime = currentTime;
context.measurementTimeCount = measurementTimeCount;
context.policyName = policyName;
context.actionName = actionName;
context.scheduleEnabled = logical(scheduleEnabled);
context.developmentEvidenceOnly = true;
end
