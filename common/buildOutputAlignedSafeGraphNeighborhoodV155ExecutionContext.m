function context = ...
    buildOutputAlignedSafeGraphNeighborhoodV155ExecutionContext( ...
        presetName, seed, policyName)
% BUILDOUTPUTALIGNEDSAFEGRAPHNEIGHBORHOODV155EXECUTIONCONTEXT Pilot permit.

protocol = getOutputAlignedSafeGraphNeighborhoodV155Protocol();
presetIdx = find(strcmp(protocol.presetNames, presetName));
token = regexp(policyName, [ ...
    '^', protocol.candidateArmPrefix, '([0-9]+)$'], ...
    'tokens', 'once');
if numel(presetIdx) ~= 1 || isempty(token) || ...
        ~strcmp(presetName, protocol.stageAPresetName) || ...
        seed ~= protocol.stageASeed
    error('SafeGraphNeighborhoodV155:InvalidExecutionContext', ...
        'The V155 request is outside the frozen Stage-A case.');
end
candidateOrdinal = str2double(token{1});
if ~isfinite(candidateOrdinal) || ...
        candidateOrdinal < 1 || ...
        candidateOrdinal > protocol.candidateCounts(presetIdx) || ...
        mod(candidateOrdinal, 1) ~= 0
    error('SafeGraphNeighborhoodV155:InvalidExecutionContext', ...
        'The V155 candidate ordinal is invalid.');
end

context = struct();
context.contractVersion = ...
    'safe-graph-neighborhood-v155-execution-context-v1';
context.capability = 'safe-graph-neighborhood-v155-development';
context.action = 'filter-safe-graph-neighborhood-v155-development';
context.protocolId = protocol.id;
context.evidenceSplit = protocol.evidenceSplit;
context.phase = 'stage-a-arm';
context.presetName = presetName;
context.seed = seed;
context.currentTime = protocol.anchorTimes(presetIdx);
context.horizonSteps = protocol.horizon;
context.measurementTimeCount = ...
    context.currentTime + context.horizonSteps - 1;
context.policyName = policyName;
context.candidateOrdinal = candidateOrdinal;
context.trackingOutcomeScoringAuthorized = true;
context.developmentEvidenceOnly = true;
end
