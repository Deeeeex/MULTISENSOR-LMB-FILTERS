function context = ...
    buildTrackingAlignedLayeredLabelHeadroomV62ExecutionContext( ...
        actionName, layeredRoutingEnabled)
% BUILDTRACKINGALIGNEDLAYEREDLABELHEADROOMV62EXECUTIONCONTEXT H=3 only.

protocol = getTrackingAlignedLayeredLabelHeadroomV62Protocol();
if nargin < 2 || isempty(layeredRoutingEnabled)
    layeredRoutingEnabled = true;
end
if ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(layeredRoutingEnabled) || ...
        ~(islogical(layeredRoutingEnabled) || ...
          isnumeric(layeredRoutingEnabled))
    error('TrackingAlignedV62:InvalidExecutionContextInput', ...
        'The V62 action name or layered-routing flag is invalid.');
end

context = struct();
context.contractVersion = ...
    'tracking-aligned-layered-label-headroom-v62-context-v1';
context.capability = ...
    'tracking-aligned-layered-label-headroom-development';
context.action = ...
    'filter-tracking-aligned-layered-label-headroom-development';
context.protocolId = protocol.id;
context.phase = 'opened-return';
context.presetName = protocol.presetName;
context.seed = protocol.seed;
context.currentTime = protocol.currentTime;
context.measurementTimeCount = ...
    protocol.currentTime + protocol.horizonSteps - 1;
context.policyName = protocol.policyName;
context.actionName = actionName;
context.layeredRoutingEnabled = logical(layeredRoutingEnabled);
context.developmentEvidenceOnly = true;
end
