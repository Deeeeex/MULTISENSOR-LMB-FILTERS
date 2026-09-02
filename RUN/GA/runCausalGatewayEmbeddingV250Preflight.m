function [reportPath, result] = ...
        runCausalGatewayEmbeddingV250Preflight(options)
% RUNCAUSALGATEWAYEMBEDDINGV250PREFLIGHT Freeze bounded M24 banks.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getCausalGatewayEmbeddingV250Protocol();
presetName = char(getField(options, 'presetName', ...
    protocol.allowedPresets{1}));
seed = getField(options, 'seed', protocol.allowedSeeds(1));
writeReport = logical(getField(options, 'writeReport', true));
if ~ismember(presetName, protocol.allowedPresets) || ...
        ~ismember(seed, protocol.allowedSeeds) || ...
        ~islogical(writeReport) || ~isscalar(writeReport)
    error('CausalGatewayEmbeddingV250:InvalidPreflight', ...
        'V250 preflight requires its registered M24 scene and seed.');
end

gitState = resolveResearchGitState();
[~, scene] = runTemporalTaskCoupledFormationBraidV247Preflight( ...
    struct('presets', {{presetName}}, 'seed', seed, ...
        'writeReport', false, 'failOnGate', true));
config = buildDynamicTopologyScenarioConfig(presetName);
[sensors, ~] = generateMultiFormationTrajectories(config);
[targets, ~] = generateCorridorTargetTrajectories(config);
graph = buildDynamicTopologyGraphs(config, sensors);
validation = validateDynamicTopologyScenario( ...
    config, sensors, targets, graph, struct('throwOnInvalid', false));
[pDrop, ~] = buildDynamicTopologyLinkSchedule(config, graph);
identity = buildDynamicTopologyPhysicalIdentityRegistry(config);

nodeCount = config.numberOfSensors;
formationCount = numel(unique(config.sensorGroupIds));
messageBudget = nodeCount + 2 * (formationCount - 1);
history = false(nodeCount, nodeCount, 0);
historyTimes = zeros(1, 0);
bankCells = cell(1, numel(protocol.anchorTimes));
anchorCursor = 1;
for currentTime = 1:max(protocol.anchorTimes)
    context = buildContext(config, graph, pDrop, identity, ...
        currentTime, history, historyTimes, messageBudget);
    [adjacency, details] = ...
        selectCausalMinimumFormationBackboneV242Policy(context);
    if anchorCursor <= numel(protocol.anchorTimes) && ...
            currentTime == protocol.anchorTimes(anchorCursor)
        bank = buildCausalGatewayEmbeddingCandidateBankV250( ...
            context, protocol.maximumCandidateCount);
        bank.referenceSensorAdjacency = logical(adjacency);
        bank.referenceTreeReselectionUsed = ...
            logical(details.treeReselectionUsed);
        bankCells{anchorCursor} = bank;
        anchorCursor = anchorCursor + 1;
    end
    history = cat(3, history, logical(adjacency));
    historyTimes(end + 1) = currentTime; %#ok<AGROW>
    if size(history, 3) > 2
        history = history(:, :, end-1:end);
        historyTimes = historyTimes(end-1:end);
    end
end
if anchorCursor ~= numel(protocol.anchorTimes) + 1
    error('CausalGatewayEmbeddingV250:MissingAnchor', ...
        'At least one registered anchor did not produce a gateway bank.');
end
banks = [bankCells{:}];

result = struct();
result.contractVersion = ...
    'causal-gateway-embedding-v250-preflight-v1';
result.generatedAt = datestr(now, 31);
result.generationGitCommit = gitState.commit;
result.protocol = protocol;
result.presetName = presetName;
result.seed = seed;
result.sceneGatePassed = scene.crossScaleGatePassed;
result.sceneValidationPassed = validation.isValid;
result.nodeCount = nodeCount;
result.formationCount = formationCount;
result.minimumMessageCount = messageBudget;
result.anchorTimes = protocol.anchorTimes;
result.banks = banks;
result.candidateCountByAnchor = [banks.candidateCount];
result.rawGlobalAssignmentCountByAnchor = ...
    [banks.rawGlobalAssignmentCount];
result.receiverCoverageByAnchor = vertcat( ...
    banks.receiverCoverageByFormation);
result.minimumReceiverCoverageByAnchor = ...
    [banks.minimumReceiverCoverage];
result.gatePassed = result.sceneGatePassed && ...
    result.sceneValidationPassed && ...
    all(result.candidateCountByAnchor >= ...
        protocol.minimumCandidateCount) && ...
    all(result.candidateCountByAnchor <= ...
        protocol.maximumCandidateCount) && ...
    all(result.minimumReceiverCoverageByAnchor >= ...
        protocol.minimumReceiverCoveragePerFormation) && ...
    all([banks.hardGatePassed]);
result.h3TrackingOracleAuthorized = result.gatePassed;
result.ridgeTrainingAuthorized = false;
result.gnnTrainingAuthorized = false;
result.nextMethodDecision = ...
    'run-paired-h3-gateway-embedding-oracle-before-learning';
result.truthUsedForCandidateConstruction = false;
result.posteriorUsedForCandidateConstruction = false;
result.futurePhysicalPageUsedForCandidateConstruction = false;
result.trackingOutcomeAuthorized = result.gatePassed;
result.developmentEvidenceOnly = true;
result.validationClaimAllowed = false;
result.evidenceBoundary = protocol.evidenceBoundary;

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outputRoot = char(getField(options, 'outputRoot', fullfile( ...
    protocol.outputRoot, 'm24_seed1301_preflight')));
if ~isAbsolutePath(outputRoot)
    outputRoot = fullfile(repoRoot, outputRoot);
end
reportPath = '';
if writeReport
    if exist(outputRoot, 'dir') ~= 7, mkdir(outputRoot); end
    reportPath = fullfile(outputRoot, ...
        'CAUSAL_GATEWAY_EMBEDDING_V250_PREFLIGHT.md');
    matPath = strrep(reportPath, '.md', '.mat');
    result.reportPath = reportPath;
    result.matPath = matPath;
    save('-mat7-binary', matPath, 'result');
    writeReportFile(reportPath, result);
end
if ~result.gatePassed
    error('CausalGatewayEmbeddingV250:PreflightGate', ...
        'The V250 M24 gateway bank violated a structural gate.');
end
fprintf('V250 preflight candidates by anchor %s: %s\n', ...
    mat2str(result.anchorTimes), ...
    mat2str(result.candidateCountByAnchor));
fprintf('V250 receiver coverage by anchor:\n');
disp(result.receiverCoverageByAnchor);
if ~isempty(reportPath)
    fprintf('V250 preflight: %s\n', reportPath);
end
end

function context = buildContext(config, graph, pDrop, identity, ...
        currentTime, history, historyTimes, messageBudget)
nodeCount = config.numberOfSensors;
context = struct();
context.localPosteriorBySensor = repmat({struct([])}, 1, nodeCount);
context.model = struct('dynamicTopologyScenario', struct( ...
    'config', config, ...
    'staticAdjacency', logical(graph.staticAdjacency)));
context.baseAdjacency = logical(graph.staticAdjacency);
context.physicalAdjacency = logical( ...
    graph.physicalAdjacency(:, :, currentTime));
context.positions = graph.positions(:, :, currentTime);
context.currentTime = currentTime;
context.commConfig = struct('pDropByEdge', pDrop(:, :, currentTime));
context.directedMessageBudget = messageBudget;
context.sensorPhysicalUids = identity.sensorPhysicalUids;
context.formationPhysicalUidsBySensor = ...
    identity.formationPhysicalUidsBySensor;
context.previousAdjacencyHistory = history;
context.previousAdjacencyHistoryCount = size(history, 3);
context.previousAdjacencyHistoryTimes = historyTimes;
end

function writeReportFile(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('CausalGatewayEmbeddingV250:ReportOpen', ...
        'Could not write the V250 preflight report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V250 causal sensor-gateway embedding preflight\n\n');
fprintf(fid, '- Scene / seed: `%s / %d`\n', ...
    result.presetName, result.seed);
fprintf(fid, '- Source commit: `%s`\n', ...
    result.generationGitCommit);
fprintf(fid, '- Scene / structural gate: `%d / %d`\n', ...
    result.sceneGatePassed, result.gatePassed);
fprintf(fid, '- N / F / exact messages: `%d / %d / %d`\n', ...
    result.nodeCount, result.formationCount, ...
    result.minimumMessageCount);
fprintf(fid, '- Candidate counts: `%s`\n', ...
    mat2str(result.candidateCountByAnchor));
fprintf(fid, '- Raw global assignment counts: `%s`\n', ...
    mat2str(result.rawGlobalAssignmentCountByAnchor));
fprintf(fid, '- Minimum receiver coverage: `%s`\n', ...
    mat2str(result.minimumReceiverCoverageByAnchor));
fprintf(fid, '- H=3 tracking oracle authorized: `%d`\n', ...
    result.h3TrackingOracleAuthorized);
fprintf(fid, ['- Candidate construction truth / posterior / future ', ...
    'physical use: `%d / %d / %d`\n\n'], ...
    result.truthUsedForCandidateConstruction, ...
    result.posteriorUsedForCandidateConstruction, ...
    result.futurePhysicalPageUsedForCandidateConstruction);
fprintf(fid, ['| Anchor | Candidates | Raw assignments | ', ...
    'Receiver coverage by formation | V242 reselection |\n']);
fprintf(fid, '|--:|--:|--:|:--|:--:|\n');
for bankIdx = 1:numel(result.banks)
    bank = result.banks(bankIdx);
    fprintf(fid, '| %d | %d | %.0f | `%s` | %d |\n', ...
        bank.currentTime, bank.candidateCount, ...
        bank.rawGlobalAssignmentCount, ...
        mat2str(bank.receiverCoverageByFormation), ...
        bank.referenceTreeReselectionUsed);
end
fprintf(fid, '\n## Candidate composition\n\n');
for bankIdx = 1:numel(result.banks)
    bank = result.banks(bankIdx);
    types = {bank.candidates.candidateType};
    uniqueTypes = unique(types, 'stable');
    parts = cell(1, numel(uniqueTypes));
    for typeIdx = 1:numel(uniqueTypes)
        parts{typeIdx} = sprintf('%s=%d', uniqueTypes{typeIdx}, ...
            nnz(strcmp(types, uniqueTypes{typeIdx})));
    end
    fprintf(fid, '- `t=%d`: %s\n', bank.currentTime, ...
        strjoin(parts, '; '));
end
fprintf(fid, '\n## Method decision\n\n');
fprintf(fid, ['The corrected M24 formation tree is fixed, but its ', ...
    'sensor-level gateway embedding has a large executable action space. ', ...
    'The bounded causal bank passes the exact-message, physicality, ', ...
    'strong-connectivity, KLA-weight and receiver-diversity gates. ', ...
    'A paired H=3 tracking oracle is therefore authorized; ridge or GNN ', ...
    'training remains unauthorized until that oracle shows repeatable ', ...
    'joint value.\n\n']);
fprintf(fid, '## Evidence boundary\n\n%s\n', ...
    result.evidenceBoundary);
end

function absolute = isAbsolutePath(path)
absolute = ~isempty(path) && (path(1) == '/' || ...
    (~isempty(regexp(path, '^[A-Za-z]:[\\/]', 'once'))));
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
