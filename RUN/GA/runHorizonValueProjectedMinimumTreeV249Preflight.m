function [reportPath, result] = ...
        runHorizonValueProjectedMinimumTreeV249Preflight(options)
% RUNHORIZONVALUEPROJECTEDMINIMUMTREEV249PREFLIGHT Freeze M24 tree banks.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getHorizonValueProjectedMinimumTreeV249Protocol();
presetName = char(getField(options, 'presetName', ...
    protocol.allowedPresets{1}));
seed = getField(options, 'seed', protocol.allowedSeeds(1));
writeReport = logical(getField(options, 'writeReport', true));
if ~ismember(presetName, protocol.allowedPresets) || ...
        ~ismember(seed, protocol.allowedSeeds) || ...
        ~islogical(writeReport) || ~isscalar(writeReport)
    error('HorizonValueProjectedMinimumTreeV249:InvalidPreflight', ...
        'V249 preflight requires its registered M24 scene and seed.');
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
        bank = enumerateHorizonValueProjectedMinimumTreesV249( ...
            context, details.currentFormationTreePairs, ...
            protocol.maximumCandidateCount);
        bank.referenceSensorAdjacency = logical(adjacency);
        bank.referenceMessageCount = nnz(adjacency);
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
    error('HorizonValueProjectedMinimumTreeV249:MissingAnchor', ...
        'At least one registered anchor did not produce a tree bank.');
end
banks = [bankCells{:}];

result = struct();
result.contractVersion = ...
    'horizon-value-projected-minimum-tree-v249-preflight-v1';
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
result.feasibleTreeCountByAnchor = [banks.feasibleTreeCount];
result.alternativeTreeCountByAnchor = ...
    result.feasibleTreeCountByAnchor - 1;
result.referenceCandidateIndexByAnchor = ...
    [banks.referenceCandidateIndex];
result.gatePassed = result.sceneGatePassed && ...
    result.sceneValidationPassed && ...
    all(result.feasibleTreeCountByAnchor >= 1) && ...
    all(arrayfun(@bankHardGate, banks));
result.nontrivialTreeActionSpacePassed = ...
    all(result.alternativeTreeCountByAnchor >= 1);
result.h3TrackingOracleAuthorized = ...
    result.gatePassed && result.nontrivialTreeActionSpacePassed;
result.gnnTrainingAuthorized = false;
result.nextMethodDecision = ...
    'terminate-formation-tree-choice-pivot-to-sensor-gateway-embedding';
result.truthUsedForCandidateConstruction = false;
result.futurePhysicalPageUsedForCandidateConstruction = false;
result.trackingOutcomeAuthorized = false;
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
        'HORIZON_VALUE_PROJECTED_MINIMUM_TREE_V249_PREFLIGHT.md');
    matPath = strrep(reportPath, '.md', '.mat');
    result.reportPath = reportPath;
    result.matPath = matPath;
    save('-mat7-binary', matPath, 'result');
    writeReportFile(reportPath, result);
end
if ~result.gatePassed
    error('HorizonValueProjectedMinimumTreeV249:PreflightGate', ...
        'The V249 M24 tree bank violated a structural gate.');
end
fprintf('V249 preflight trees by anchor %s: %s\n', ...
    mat2str(result.anchorTimes), ...
    mat2str(result.feasibleTreeCountByAnchor));
if ~isempty(reportPath)
    fprintf('V249 preflight: %s\n', reportPath);
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

function passed = bankHardGate(bank)
expectedMessages = size(bank.candidates(1).sensorAdjacency, 1) + ...
    2 * (numel(bank.formationPhysicalUids) - 1);
passed = bank.referenceCandidateIndex >= 1 && ...
    all([bank.candidates.messageCount] == expectedMessages) && ...
    all([bank.candidates.stronglyConnected]) && ...
    all([bank.candidates.physicallyFeasible]) && ...
    all([bank.candidates.weightsValid]) && ...
    ~bank.truthUsed && ~bank.posteriorUsed && ...
    ~bank.futurePhysicalPageUsed;
end

function writeReportFile(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('HorizonValueProjectedMinimumTreeV249:ReportOpen', ...
        'Could not write the V249 preflight report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V249 H=3 minimum-tree oracle preflight\n\n');
fprintf(fid, '- Scene / seed: `%s / %d`\n', ...
    result.presetName, result.seed);
fprintf(fid, '- Source commit: `%s`\n', ...
    result.generationGitCommit);
fprintf(fid, '- Scene / structural gate: `%d / %d`\n', ...
    result.sceneGatePassed, result.gatePassed);
fprintf(fid, '- N / F / exact messages: `%d / %d / %d`\n', ...
    result.nodeCount, result.formationCount, ...
    result.minimumMessageCount);
fprintf(fid, '- Alternative trees by anchor: `%s`\n', ...
    mat2str(result.alternativeTreeCountByAnchor));
fprintf(fid, '- Nontrivial action space / H=3 authorized: `%d / %d`\n', ...
    result.nontrivialTreeActionSpacePassed, ...
    result.h3TrackingOracleAuthorized);
fprintf(fid, ['- Candidate construction truth / future physical use: ', ...
    '`%d / %d`\n\n'], ...
    result.truthUsedForCandidateConstruction, ...
    result.futurePhysicalPageUsedForCandidateConstruction);
fprintf(fid, '| Anchor | Feasible trees | V242 index | V242 tree | Reselection |\n');
fprintf(fid, '|--:|--:|--:|:--|:--:|\n');
for bankIdx = 1:numel(result.banks)
    bank = result.banks(bankIdx);
    fprintf(fid, '| %d | %d | %d | `%s` | %d |\n', ...
        bank.currentTime, bank.feasibleTreeCount, ...
        bank.referenceCandidateIndex, ...
        pairString(bank.referenceFormationUidPairs), ...
        bank.referenceTreeReselectionUsed);
end
fprintf(fid, '\n## Frozen candidate banks\n\n');
for bankIdx = 1:numel(result.banks)
    bank = result.banks(bankIdx);
    fprintf(fid, '**t=%d.** ', bank.currentTime);
    for candidateIdx = 1:numel(bank.candidates)
        marker = '';
        if candidateIdx == bank.referenceCandidateIndex
            marker = ' (V242)';
        end
        fprintf(fid, '`T%02d=%s%s`', candidateIdx, ...
            pairString(bank.candidates(candidateIdx).formationUidPairs), ...
            marker);
        if candidateIdx < numel(bank.candidates)
            fprintf(fid, '; ');
        end
    end
    fprintf(fid, '\n\n');
end
fprintf(fid, '\n## Method decision\n\n');
fprintf(fid, ['Each registered physical formation graph contains exactly ', ...
    'F-1 edges and therefore has one spanning tree. Formation-tree ', ...
    'selection has no action at the task-coupled cut windows, so the ', ...
    'H=3 tracking oracle and GNN training are stopped before execution. ', ...
    'The next action space should keep this formation skeleton and select ', ...
    'its sensor-level gateway embedding under the same message budget.\n\n']);
fprintf(fid, '## Evidence boundary\n\n%s\n', ...
    result.evidenceBoundary);
end

function value = pairString(pairs)
parts = cell(1, size(pairs, 1));
for idx = 1:size(pairs, 1)
    parts{idx} = sprintf('%d-%d', pairs(idx, 1), pairs(idx, 2));
end
value = strjoin(parts, ',');
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
