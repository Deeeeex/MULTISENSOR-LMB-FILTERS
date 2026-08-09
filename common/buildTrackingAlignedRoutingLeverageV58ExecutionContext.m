function context = ...
    buildTrackingAlignedRoutingLeverageV58ExecutionContext( ...
        presetName, seed, currentTime)
% BUILDTRACKINGALIGNEDROUTINGLEVERAGEV58EXECUTIONCONTEXT Reference only.

protocol = getTrackingAlignedRoutingLeverageV58Protocol();
entry = getTrackingAlignedRoutingLeverageV58Case(presetName, seed);
if ~isa(currentTime, 'double') || ~isreal(currentTime) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= floor(currentTime) || ...
        currentTime ~= max(entry.snapshotTimes)
    error('TrackingAlignedV58:InvalidReferenceWindow', ...
        'The V58 reference run must cover its complete registered case.');
end

context = struct();
context.contractVersion = ...
    'tracking-aligned-routing-leverage-v58-execution-context-v1';
context.capability = ...
    'tracking-aligned-routing-leverage-reference-development';
context.action = ...
    'filter-routing-leverage-reference-development';
context.protocolId = protocol.id;
context.phase = 'reference-cache';
context.caseId = entry.caseId;
context.presetName = presetName;
context.seed = seed;
context.currentTime = currentTime;
context.measurementTimeCount = currentTime;
context.policyName = protocol.referencePolicyName;
context.developmentEvidenceOnly = true;
end
