function context = ...
        buildOneShotPacketEpisodeV270ExecutionContext( ...
            presetName, seed, measurementTimeCount)
% BUILDONESHOTPACKETEPISODEV270EXECUTIONCONTEXT Dev permit.

protocol = getOneShotPacketEpisodeV270Protocol();
if ~ischar(presetName) || ...
        ~ismember(presetName, protocol.allowedPresets) || ...
        ~ismember(seed, protocol.allowedSeeds) || ...
        measurementTimeCount ~= protocol.continuationEndTime
    error('OneShotPacketEpisodeV270:InvalidContextInput', ...
        'V270 requires its registered continuation.');
end
context = struct();
context.contractVersion = 'one-shot-packet-episode-v270-context-v1';
context.capability = 'one-shot-packet-episode-v270-development';
context.action = 'filter-one-shot-packet-episode-v270-development';
context.protocolId = protocol.id;
context.presetName = presetName;
context.seed = seed;
context.armId = protocol.armId;
context.measurementTimeCount = measurementTimeCount;
context.continuationStartTime = protocol.continuationStartTime;
context.policyName = protocol.policyName;
context.developmentEvidenceOnly = true;
end
