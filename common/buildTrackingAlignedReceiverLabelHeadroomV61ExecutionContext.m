function context = ...
    buildTrackingAlignedReceiverLabelHeadroomV61ExecutionContext( ...
        actionName, labelRoutingEnabled)
% BUILDTRACKINGALIGNEDRECEIVERLABELHEADROOMV61EXECUTIONCONTEXT H=3 only.

protocol = getTrackingAlignedReceiverLabelHeadroomV61Protocol();
if nargin < 2 || isempty(labelRoutingEnabled)
    labelRoutingEnabled = true;
end
if ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(labelRoutingEnabled) || ...
        ~(islogical(labelRoutingEnabled) || isnumeric(labelRoutingEnabled))
    error('TrackingAlignedV61:InvalidExecutionContextInput', ...
        'The V61 action name or label-routing flag is invalid.');
end

context = struct();
context.contractVersion = ...
    'tracking-aligned-receiver-label-headroom-v61-context-v1';
context.capability = ...
    'tracking-aligned-receiver-label-headroom-development';
context.action = ...
    'filter-tracking-aligned-receiver-label-headroom-development';
context.protocolId = protocol.id;
context.phase = 'opened-return';
context.presetName = protocol.presetName;
context.seed = protocol.seed;
context.currentTime = protocol.currentTime;
context.measurementTimeCount = ...
    protocol.currentTime + protocol.horizonSteps - 1;
context.policyName = protocol.policyName;
context.actionName = actionName;
context.labelRoutingEnabled = logical(labelRoutingEnabled);
context.developmentEvidenceOnly = true;
end
