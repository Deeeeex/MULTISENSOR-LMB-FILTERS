function [reportPath, result] = ...
    generateInfluenceAwareDecisionBreadthV66RelayCaches(options)
% GENERATEINFLUENCEAWAREDECISIONBREADTHV66RELAYCACHES One source run.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = ...
    getInfluenceAwareDecisionBreadthV66SceneDiscoveryProtocol();
outputRoot = getField(options, 'outputRoot', protocol.outputRoot);
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
[cachePaths, summary] = generateFormationH3ReferenceStateCaches( ...
    protocol.presets{1}, protocol.allSeeds, protocol.snapshotTimes, ...
    struct('protocol', protocol, 'cacheRoot', protocol.cacheRoot));
result = struct();
result.contractVersion = ...
    'influence-aware-decision-breadth-v66-relay-cache-result-v1';
result.protocol = protocol;
result.summary = summary;
result.cachePaths = cachePaths;
result.trackingOutcomeScored = false;
result.modelTrainingAuthorized = false;
result.validationClaimAllowed = false;
result.evidenceBoundary = protocol.evidenceBoundary;
matPath = fullfile(outputRoot, ...
    'INFLUENCE_AWARE_DECISION_BREADTH_V66_RELAY_CACHE_RESULT.mat');
reportPath = fullfile(outputRoot, ...
    'INFLUENCE_AWARE_DECISION_BREADTH_V66_RELAY_CACHE_RESULT.md');
result.matPath = matPath;
result.reportPath = reportPath;
save('-mat7-binary', matPath, 'result');
writeReport(reportPath, result);
fprintf('V66 relay reference caches: %s\n', reportPath);
end

function writeReport(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('InfluenceAwareRelayV66:WriteFailed', ...
        'Could not create the V66 relay cache report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V66 M24 relay reference caches\n\n');
fprintf(fid, '- Preset / seed: `%s / %d`\n', ...
    result.protocol.presets{1}, result.protocol.allSeeds);
fprintf(fid, '- Snapshot times: `%s`\n', ...
    mat2str(result.protocol.snapshotTimes));
fprintf(fid, '- Cache count: `%d`\n', numel(result.cachePaths));
fprintf(fid, '- Reused existing: `%d`\n', ...
    result.summary.reusedExisting);
fprintf(fid, '- Tracking outcome scored: `0`\n');
fprintf(fid, '\n## Evidence boundary\n\n%s\n', ...
    result.evidenceBoundary);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
