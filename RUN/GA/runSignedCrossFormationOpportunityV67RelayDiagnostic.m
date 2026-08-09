function [reportPath, result] = ...
    runSignedCrossFormationOpportunityV67RelayDiagnostic(options)
% RUNSIGNEDCROSSFORMATIONOPPORTUNITYV67RELAYDIAGNOSTIC Source-only scan.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getSignedCrossFormationOpportunityV67Protocol();
v65 = getNetworkAdditiveFormationRiskV65Protocol();
gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles) || ...
        gitState.researchWorktreeDirty
    error('SignedOpportunityV67:DirtySource', ...
        'The V67 source-only diagnostic requires clean source.');
end
outputRoot = getField(options, 'outputRoot', protocol.outputRoot);
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end

times = protocol.snapshotTimes;
records = repmat(emptyRecord(), 1, numel(times));
for timeIdx = 1:numel(times)
    currentTime = times(timeIdx);
    state = loadFormationH3ObservableState( ...
        protocol.presets{1}, protocol.allSeeds, currentTime, ...
        struct('cacheRoot', protocol.cacheRoot));
    [~, control] = computeFormationRoutingLeverageSignature( ...
        state.context, state.groupIds);
    baseMetrics = computeNetworkAdditiveFormationRiskV65( ...
        control, state.context, state.groupIds, ...
        v65.positiveSupportThreshold);
    signed = computeSignedCrossFormationOpportunityV67( ...
        baseMetrics, protocol.minimumMaterialPressureFraction);
    records(timeIdx) = makeRecord(currentTime, signed);
    fprintf(['V67 relay t=%d quarantine=%.3f%% transport=%.3f%% ', ...
        'balance=%+.3f%% regime=%s\n'], ...
        currentTime, 100 * signed.totalQuarantinePressure, ...
        100 * signed.totalTransportRetentionPressure, ...
        100 * signed.totalSignedTransportBalance, signed.regime);
end

[~, maxTransportIdx] = max([records.transportRetentionPressure]);
[~, maxQuarantineIdx] = max([records.quarantinePressure]);
result = struct();
result.contractVersion = ...
    'signed-cross-formation-opportunity-v67-relay-result-v1';
result.protocol = protocol;
result.generatedAt = datestr(now, 31);
result.generationGitCommit = gitState.commit;
result.records = records;
result.maximumTransportRecord = records(maxTransportIdx);
result.maximumQuarantineRecord = records(maxQuarantineIdx);
result.materialTransportTimeCount = nnz(strcmp( ...
    {records.regime}, 'transport') | strcmp({records.regime}, 'mixed'));
result.materialQuarantineTimeCount = nnz(strcmp( ...
    {records.regime}, 'quarantine') | strcmp({records.regime}, 'mixed'));
result.trackingOutcomeRead = false;
result.trackingOutcomeScoringAuthorized = false;
result.modelTrainingAuthorized = false;
result.validationClaimAllowed = false;
result.openedDevelopmentEvidenceOnly = true;
result.evidenceBoundary = protocol.evidenceBoundary;

matPath = fullfile(outputRoot, ...
    'SIGNED_CROSS_FORMATION_OPPORTUNITY_V67_RELAY.mat');
reportPath = fullfile(outputRoot, ...
    'SIGNED_CROSS_FORMATION_OPPORTUNITY_V67_RELAY.md');
result.matPath = matPath;
result.reportPath = reportPath;
save('-mat7-binary', matPath, 'result');
writeReport(reportPath, result);
fprintf('V67 signed relay diagnostic: %s\n', reportPath);
end

function record = makeRecord(currentTime, metrics)
record = emptyRecord();
record.currentTime = currentTime;
record.quarantinePressure = metrics.totalQuarantinePressure;
record.transportRetentionPressure = ...
    metrics.totalTransportRetentionPressure;
record.signedTransportBalance = metrics.totalSignedTransportBalance;
record.transportShare = metrics.transportShare;
record.regime = metrics.regime;
end

function value = emptyRecord()
value = struct( ...
    'currentTime', NaN, ...
    'quarantinePressure', NaN, ...
    'transportRetentionPressure', NaN, ...
    'signedTransportBalance', NaN, ...
    'transportShare', NaN, ...
    'regime', 'not-evaluated');
end

function writeReport(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('SignedOpportunityV67:WriteFailed', ...
        'Could not create the V67 signed-opportunity report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V67 signed cross-formation relay diagnostic\n\n');
fprintf(fid, ['| Time | Quarantine pressure | Transport-retention ', ...
    'pressure | Signed transport balance | Transport share | Regime |\n']);
fprintf(fid, '|--:|--:|--:|--:|--:|:--|\n');
for record = result.records
    fprintf(fid, '| %d | %.3f%% | %.3f%% | %+.3f%% | %.3f | `%s` |\n', ...
        record.currentTime, 100 * record.quarantinePressure, ...
        100 * record.transportRetentionPressure, ...
        100 * record.signedTransportBalance, ...
        record.transportShare, record.regime);
end
fprintf(fid, '\n- Material transport times: `%d`\n', ...
    result.materialTransportTimeCount);
fprintf(fid, '- Material quarantine times: `%d`\n', ...
    result.materialQuarantineTimeCount);
fprintf(fid, '- Maximum transport time / pressure: `%d / %.3f%%`\n', ...
    result.maximumTransportRecord.currentTime, ...
    100 * result.maximumTransportRecord.transportRetentionPressure);
fprintf(fid, '- Maximum quarantine time / pressure: `%d / %.3f%%`\n', ...
    result.maximumQuarantineRecord.currentTime, ...
    100 * result.maximumQuarantineRecord.quarantinePressure);
fprintf(fid, '- Tracking outcome read: `0`\n');
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
