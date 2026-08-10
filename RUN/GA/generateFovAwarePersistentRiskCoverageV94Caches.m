function [reportPath, result] = ...
    generateFovAwarePersistentRiskCoverageV94Caches(options)
% GENERATEFOVAWAREPERSISTENTRISKCOVERAGEV94CACHES Matched V94 states.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getFovAwarePersistentRiskCoverageV94Protocol();
overwrite = logical(getField(options, 'overwrite', false));
gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles) || ...
        gitState.researchWorktreeDirty
    error('FovAwareV94:DirtySource', ...
        'V94 cache generation requires a clean source checkout.');
end

caseResults = repmat(struct( ...
    'scaleName', '', 'presetName', '', 'seed', NaN, ...
    'cacheGenerationTimes', [], 'cachePaths', {{}}, ...
    'cacheRoot', '', 'sourceProtocolId', '', ...
    'missingLabelFusionMode', '', 'reusedExisting', false), ...
    1, numel(protocol.cases));
for caseIdx = 1:numel(protocol.cases)
    entry = protocol.cases(caseIdx);
    sourceProtocol = resolveSourceProtocol(entry.sourceProtocolId);
    cacheOptions = struct( ...
        'protocol', sourceProtocol, ...
        'cacheRoot', entry.cacheRoot, ...
        'overwrite', overwrite, ...
        'receiverMode', protocol.receiverMode);
    [cachePaths, summary] = ...
        generateFormationH3ReferenceStateCaches( ...
            entry.presetName, entry.seed, ...
            entry.cacheGenerationTimes, cacheOptions);
    caseResults(caseIdx).scaleName = entry.scaleName;
    caseResults(caseIdx).presetName = entry.presetName;
    caseResults(caseIdx).seed = entry.seed;
    caseResults(caseIdx).cacheGenerationTimes = ...
        entry.cacheGenerationTimes;
    caseResults(caseIdx).cachePaths = cachePaths;
    caseResults(caseIdx).cacheRoot = entry.cacheRoot;
    caseResults(caseIdx).sourceProtocolId = entry.sourceProtocolId;
    caseResults(caseIdx).missingLabelFusionMode = ...
        summary.missingLabelFusionMode;
    caseResults(caseIdx).reusedExisting = summary.reusedExisting;
end

result = struct();
result.contractVersion = ...
    'fov-aware-persistent-risk-coverage-v94-cache-result-v1';
result.protocolId = protocol.id;
result.generatedAt = datestr(now, 31);
result.generationGitCommit = gitState.commit;
result.baselineName = protocol.baselineName;
result.missingLabelFusionMode = protocol.receiverMode;
result.cases = caseResults;
result.validationClaimAllowed = false;
result.openedDevelopmentEvidenceOnly = true;

if exist(protocol.outputRoot, 'dir') ~= 7
    mkdir(protocol.outputRoot);
end
matPath = fullfile(protocol.outputRoot, ...
    'FOV_AWARE_PERSISTENT_RISK_COVERAGE_V94_CACHE_RESULT.mat');
reportPath = fullfile(protocol.outputRoot, ...
    'FOV_AWARE_PERSISTENT_RISK_COVERAGE_V94_CACHE_RESULT.md');
result.matPath = matPath;
result.reportPath = reportPath;
save('-mat7-binary', matPath, 'result');
writeReport(reportPath, result);
fprintf('V94 FoV-aware cache result: %s\n', reportPath);
end

function protocol = resolveSourceProtocol(protocolId)
switch protocolId
    case 'tracking-aligned-routing-leverage-v58-v1'
        protocol = getTrackingAlignedRoutingLeverageV58Protocol();
    case 'tracking-aligned-x36-schedule-headroom-v63-v1'
        protocol = getTrackingAlignedX36ScheduleHeadroomV63Protocol();
    otherwise
        error('FovAwareV94:UnknownSourceProtocol', ...
            'The V94 source cache protocol is not registered.');
end
end

function writeReport(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('FovAwareV94:WriteFailed', ...
        'Could not create the V94 cache report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V94 FoV-aware matched-static caches\n\n');
fprintf(fid, '- Baseline: `%s`\n', result.baselineName);
fprintf(fid, '- Missing-label receiver mode: `%s`\n\n', ...
    result.missingLabelFusionMode);
fprintf(fid, '| Scale | Scenario / seed | Cached times | Reused |\n');
fprintf(fid, '|:--|:--|:--|:--:|\n');
for idx = 1:numel(result.cases)
    entry = result.cases(idx);
    fprintf(fid, '| %s | `%s / %d` | `%s` | %d |\n', ...
        entry.scaleName, entry.presetName, entry.seed, ...
        mat2str(entry.cacheGenerationTimes), entry.reusedExisting);
end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
