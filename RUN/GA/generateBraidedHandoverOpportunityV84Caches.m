function [reportPath, result] = ...
    generateBraidedHandoverOpportunityV84Caches(options)
% GENERATEBRAIDEDHANDOVEROPPORTUNITYV84CACHES M24/X36 source runs.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getBraidedHandoverOpportunityV84Protocol();
outputRoot = getField(options, 'outputRoot', protocol.outputRoot);
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end

allPaths = cell(1, numel(protocol.presets));
summaries = repmat(struct(), 1, numel(protocol.presets));
for presetIdx = 1:numel(protocol.presets)
    [allPaths{presetIdx}, summaries(presetIdx)] = ...
        generateFormationH3ReferenceStateCaches( ...
            protocol.presets{presetIdx}, protocol.allSeeds, ...
            protocol.snapshotTimes, struct( ...
                'protocol', protocol, ...
                'cacheRoot', protocol.cacheRoot));
end

result = struct();
result.contractVersion = ...
    'braided-handover-opportunity-v84-cache-result-v1';
result.protocol = protocol;
result.summaries = summaries;
result.cachePathsByPreset = allPaths;
result.trackingOutcomeScored = false;
result.modelTrainingAuthorized = false;
result.validationClaimAllowed = false;
result.evidenceBoundary = protocol.evidenceBoundary;
matPath = fullfile(outputRoot, ...
    'BRAIDED_HANDOVER_OPPORTUNITY_V84_CACHE_RESULT.mat');
reportPath = fullfile(outputRoot, ...
    'BRAIDED_HANDOVER_OPPORTUNITY_V84_CACHE_RESULT.md');
result.matPath = matPath;
result.reportPath = reportPath;
save('-mat7-binary', matPath, 'result');
writeReport(reportPath, result);
fprintf('V84 braided-handover reference caches: %s\n', reportPath);
end

function writeReport(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('BraidedHandoverV84:WriteFailed', ...
        'Could not create the V84 cache report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V84 M24/X36 braided-handover reference caches\n\n');
fprintf(fid, '| Preset | Seed | Snapshot count | Reused | Seconds |\n');
fprintf(fid, '|:--|--:|--:|:--:|--:|\n');
for idx = 1:numel(result.summaries)
    summary = result.summaries(idx);
    fprintf(fid, '| `%s` | %d | %d | %d | %.1f |\n', ...
        summary.presetName, summary.seed, ...
        numel(summary.snapshotTimes), summary.reusedExisting, ...
        summary.elapsedSeconds);
end
fprintf(fid, '\n- Tracking outcome scored: `0`\n');
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
