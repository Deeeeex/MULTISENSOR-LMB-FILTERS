function [reportPath, result] = ...
    runAlternativeTransportHeadroomV68RelayT124(options)
% RUNALTERNATIVETRANSPORTHEADROOMV68RELAYT124 One source-only state.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getAlternativeTransportHeadroomV68Protocol();
gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles) || ...
        gitState.researchWorktreeDirty
    error('AlternativeTransportV68:DirtySource', ...
        'V68 requires a clean source checkout.');
end
outputRoot = getField(options, 'outputRoot', protocol.outputRoot);
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
state = loadFormationH3ObservableState( ...
    protocol.presetName, protocol.seed, protocol.currentTime, ...
    struct('cacheRoot', protocol.cacheRoot));
[~, control] = computeFormationRoutingLeverageSignature( ...
    state.context, state.groupIds);
metrics = computeAlternativeTransportHeadroomV68( ...
    state.context, control, state.groupIds, protocol);

result = struct();
result.contractVersion = ...
    'alternative-transport-headroom-v68-relay-result-v1';
result.protocol = protocol;
result.generatedAt = datestr(now, 31);
result.generationGitCommit = gitState.commit;
result.metrics = metrics;
result.materialHeadroomPassed = ...
    metrics.optimisticNetHeadroomFraction >= ...
        protocol.minimumMaterialHeadroomFraction - 1e-12;
result.trackingOutcomeRead = false;
result.routeExecutionAuthorized = false;
result.trackingOutcomeScoringAuthorized = false;
result.modelTrainingAuthorized = false;
result.validationClaimAllowed = false;
result.openedDevelopmentEvidenceOnly = true;
result.evidenceBoundary = protocol.evidenceBoundary;

matPath = fullfile(outputRoot, ...
    'ALTERNATIVE_TRANSPORT_HEADROOM_V68_RELAY_T124.mat');
reportPath = fullfile(outputRoot, ...
    'ALTERNATIVE_TRANSPORT_HEADROOM_V68_RELAY_T124.md');
result.matPath = matPath;
result.reportPath = reportPath;
save('-mat7-binary', matPath, 'result');
writeReport(reportPath, result);
fprintf('V68 alternative transport headroom: %s\n', reportPath);
end

function writeReport(path, result)
m = result.metrics;
fid = fopen(path, 'w');
if fid < 0
    error('AlternativeTransportV68:WriteFailed', ...
        'Could not create the V68 report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V68 alternative transport headroom at relay t=124\n\n');
fprintf(fid, '- Residual slots checked: `%d`\n', m.residualSlotCount);
fprintf(fid, '- Safe positive slots: `%d`\n', m.safePositiveSlotCount);
fprintf(fid, '- Optimistic transport gain: `%.3f%%`\n', ...
    100 * m.optimisticTransportGainFraction);
fprintf(fid, '- Supported harm: `%.3f%%`\n', ...
    100 * m.optimisticSupportedHarmFraction);
fprintf(fid, '- Optimistic net headroom: `%.3f%%`\n', ...
    100 * m.optimisticNetHeadroomFraction);
fprintf(fid, '- Material 1%% headroom passed: `%d`\n', ...
    result.materialHeadroomPassed);
fprintf(fid, '- Connectivity projected: `0`\n');
fprintf(fid, '- Tracking outcome read: `0`\n\n');
fprintf(fid, ['| Receiver | Incumbent sender | Alternative sender | ', ...
    'Reliability | Transport | Harm | Net | Up | Down |\n']);
fprintf(fid, '|--:|--:|--:|--:|--:|--:|--:|--:|--:|\n');
for slot = m.slotRecords
    if ~slot.bestSafe
        continue;
    end
    fprintf(fid, ['| %d | %d | %d | %.3f | %.3f%% | %.3f%% | ', ...
        '%+.3f%% | %.0f | %.0f |\n'], ...
        slot.receiverIdx, slot.incumbentSenderIdx, ...
        slot.bestCandidateSenderIdx, slot.bestLinkReliability, ...
        100 * slot.bestTransportGainFraction, ...
        100 * slot.bestSupportedHarmFraction, ...
        100 * slot.bestNetHeadroomFraction, ...
        slot.bestUpwardCrossingCount, ...
        slot.bestDownwardCrossingCount);
end
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
