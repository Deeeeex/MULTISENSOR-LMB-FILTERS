function [auditPath, audit] = ...
    auditBackboneResidualSplicedCycleHeadroom( ...
        sourcePath, options)
% AUDITBACKBONERESIDUALSPLICEDCYCLEHEADROOM Fail-closed M24 audit.

if nargin < 2 || isempty(options)
    options = struct();
end
protocol = getBackboneResidualSplicedCycleProtocol();
if nargin < 1 || isempty(sourcePath)
    inputDirectory = getField(options, ...
        'inputDirectory', fullfile( ...
            'RUN', 'GA', 'dynamic_topology', 'evidence', ...
            'backbone_residual_spliced_cycle', ...
            'm24_hard_seed7'));
    files = dir(fullfile(inputDirectory, ...
        'DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_*.mat'));
    if numel(files) ~= 1
        error('Expected exactly one spliced-cycle source.');
    end
    sourcePath = fullfile(files(1).folder, files(1).name);
end
loaded = load(sourcePath, 'summary');
if ~isfield(loaded, 'summary')
    error('Spliced-cycle source lacks summary.');
end
summary = loaded.summary;
validateSummary(summary, protocol);
aggregates = summary.aggregate;
local = requireArm(aggregates, 'local');
baselines = arrayfun(@(idx) ...
    requireArm(aggregates, protocol.baselineArms{idx}), ...
    1:numel(protocol.baselineArms));
candidate = requireArm(aggregates, ...
    protocol.currentOracleArm);
matchedStatic = requireArm(aggregates, ...
    'backbone-residual-static-a70-e05');

[~, ~, sourceState] = ...
    loadBackboneResidualReturnHeadroomState();
selectedDiagnostic = ...
    candidate.focusSelectedDirectedEdgeHistoryBySeed{1};
selectedPolicy = ...
    convertDiagnosticEdgeHistoryToPolicyAdjacencyHistory( ...
        selectedDiagnostic);
[sensorPass, formationPass] = computeRollingPass( ...
    sourceState.previousAdjacencyHistory, ...
    selectedPolicy, sourceState.groupIds);

meanGains = arrayfun(@(baseline) ...
    fractionalGain(baseline.focusEospa, ...
        candidate.focusEospa), baselines);
tailGains = arrayfun(@(baseline) ...
    fractionalGain( ...
        baseline.focusWorstSensorEospa, ...
        candidate.focusWorstSensorEospa), baselines);
localMeanGain = fractionalGain( ...
    local.focusEospa, candidate.focusEospa);
localTailGain = fractionalGain( ...
    local.focusWorstSensorEospa, ...
    candidate.focusWorstSensorEospa);
byteDeviation = fractionalDeviation( ...
    matchedStatic.focusAttemptedBytes, ...
    candidate.focusAttemptedBytes);
repairRate = finiteOrZero(candidate.policyRepairRate);
emergencyRate = finiteOrZero( ...
    candidate.policyPayloadEmergencyRate);
infeasibleRate = finiteOrZero( ...
    candidate.focusTopologyInfeasibleRate);
truthUseFraction = finiteOrZero( ...
    candidate.policyTruthUseFraction);
crossEdgeCount = finiteOrZero( ...
    candidate.meanPolicyCrossFormationMessageCount);

gates = struct();
gates.mean = min(meanGains) + 1e-12 >= ...
    protocol.minimumMeanTrackingGainFraction && ...
    localMeanGain >= -1e-12;
gates.tail = min(tailGains) >= ...
    -protocol.maximumWorstNodeRegressionFraction - 1e-12 && ...
    localTailGain >= -1e-12;
gates.bytes = byteDeviation <= ...
    protocol.maximumAttemptedByteDeviationFraction + 1e-12;
gates.safety = all(sensorPass) && ...
    all(formationPass) && repairRate == 0 && ...
    emergencyRate == 0 && infeasibleRate == 0;
gates.provenance = ...
    abs(truthUseFraction - 1) <= 1e-12;
groupCount = numel(unique( ...
    sourceState.groupIds, 'stable'));
gates.crossCount = ...
    abs(crossEdgeCount - groupCount) <= 1e-12;

gitState = resolveResearchGitState();
audit = struct();
audit.contractVersion = ...
    'backbone-residual-spliced-cycle-headroom-audit-v1';
audit.protocolId = protocol.id;
audit.generatedAt = datestr(now, 31);
audit.sourcePath = sourcePath;
audit.sourceSha256 = computeFileSha256(sourcePath);
audit.baselineModes = {baselines.mode};
audit.localMode = local.mode;
audit.candidateMode = candidate.mode;
audit.baselineEospa = [baselines.focusEospa];
audit.baselineWorstSensorEospa = ...
    [baselines.focusWorstSensorEospa];
audit.candidateEospa = candidate.focusEospa;
audit.candidateWorstSensorEospa = ...
    candidate.focusWorstSensorEospa;
audit.minimumBaselineMeanGainFraction = ...
    min(meanGains);
audit.minimumBaselineWorstNodeGainFraction = ...
    min(tailGains);
audit.localMeanGainFraction = localMeanGain;
audit.localWorstNodeGainFraction = localTailGain;
audit.attemptedByteDeviationFraction = ...
    byteDeviation;
audit.sensorB3Pass = sensorPass;
audit.formationB3Pass = formationPass;
audit.repairRate = repairRate;
audit.payloadEmergencyRate = emergencyRate;
audit.topologyInfeasibleRate = infeasibleRate;
audit.truthUseFraction = truthUseFraction;
audit.meanCrossFormationEdgeCount = crossEdgeCount;
audit.expectedCrossFormationEdgeCount = groupCount;
audit.gates = gates;
audit.constraintEligible = gates.bytes && ...
    gates.safety && gates.provenance && gates.crossCount;
audit.headroomGatePassed = audit.constraintEligible && ...
    gates.mean && gates.tail;
audit.returnDataGenerationAuthorized = ...
    audit.headroomGatePassed;
audit.criticTrainingAuthorized = false;
audit.x36PolicyRunAuthorized = false;
audit.heldoutClaimAllowed = false;
audit.privilegedDiagnosticOnly = true;
audit.auditGitCommit = gitState.commit;
audit.auditTrackedWorktreeDirty = ...
    gitState.trackedWorktreeDirty;
audit.auditUntrackedSourceFiles = ...
    gitState.untrackedSourceFiles;
audit.evidenceBoundary = [ ...
    'The current-risk cycle reads current truth and is a ', ...
    'nondeployable development diagnostic. The executed selected ', ...
    'topology is independently rechecked against the frozen B=3 ', ...
    'history. A pass would authorize return-data generation only.'];

outputDirectory = getField(options, ...
    'outputDirectory', fileparts(sourcePath));
if ~exist(outputDirectory, 'dir')
    mkdir(outputDirectory);
end
auditPath = fullfile(outputDirectory, ...
    'backbone_residual_spliced_cycle_audit_m24_h3_v1.mat');
reportPath = fullfile(outputDirectory, ...
    'BACKBONE_RESIDUAL_SPLICED_CYCLE_AUDIT_M24_H3_V1.md');
audit.reportPath = reportPath;
save('-mat7-binary', auditPath, 'audit');
writeReport(reportPath, audit, local, baselines);
fprintf('Audit: %s\nReport: %s\n', ...
    auditPath, reportPath);
end

function validateSummary(summary, protocol)
required = {'presetName', 'seeds', 'options', 'aggregate'};
if ~isstruct(summary) || ~all(isfield(summary, required)) || ...
        ~strcmp(summary.presetName, protocol.presetName) || ...
        ~isequal(summary.seeds, protocol.seed) || ...
        summary.options.continuationStartTime ~= ...
            protocol.continuationStartTime || ...
        summary.options.maxTimeSteps ~= ...
            protocol.continuationEndTime
    error('Spliced-cycle source violates the frozen protocol.');
end
observedModes = {summary.aggregate.mode};
if ~all(ismember(protocol.armNames, observedModes)) || ...
        numel(observedModes) ~= numel(protocol.armNames)
    error('Spliced-cycle source arm set differs from protocol.');
end
end

function arm = requireArm(aggregates, mode)
index = find(strcmp({aggregates.mode}, mode));
if numel(index) ~= 1
    error('Expected exactly one spliced-cycle arm: %s.', mode);
end
arm = aggregates(index);
end

function [sensorPass, formationPass] = ...
    computeRollingPass(previousHistory, executed, groupIds)
depth = 2;
history = logical(previousHistory);
if size(history, 3) > depth
    history = history(:, :, end-depth+1:end);
end
stepCount = size(executed, 3);
sensorPass = false(1, stepCount);
formationPass = false(1, stepCount);
for stepIdx = 1:stepCount
    pages = cat(3, history, executed(:, :, stepIdx));
    window = any(pages(:, :, ...
        max(1, end-depth):end), 3);
    sensorPass(stepIdx) = ...
        isStronglyConnected(window);
    formationPass(stepIdx) = ...
        isStronglyConnected( ...
            collapseToFormations(window, groupIds));
    history(:, :, end + 1) = executed(:, :, stepIdx);
    if size(history, 3) > depth
        history = history(:, :, end-depth+1:end);
    end
end
end

function formationAdjacency = ...
    collapseToFormations(adjacency, groupIds)
groups = unique(reshape(groupIds, 1, []), 'stable');
formationAdjacency = false(numel(groups));
for receiverGroupIdx = 1:numel(groups)
    receivers = groupIds == groups(receiverGroupIdx);
    for senderGroupIdx = 1:numel(groups)
        senders = groupIds == groups(senderGroupIdx);
        formationAdjacency(receiverGroupIdx, senderGroupIdx) = ...
            any(any(adjacency(receivers, senders)));
    end
end
formationAdjacency(1:numel(groups)+1:end) = false;
end

function valid = isStronglyConnected(adjacency)
valid = true;
for startNode = 1:size(adjacency, 1)
    visited = false(1, size(adjacency, 1));
    visited(startNode) = true;
    while true
        next = visited | any(adjacency(visited, :), 1);
        if isequal(next, visited)
            break;
        end
        visited = next;
    end
    if ~all(visited)
        valid = false;
        return;
    end
end
end

function gain = fractionalGain(reference, candidate)
gain = (reference - candidate) / max(abs(reference), eps);
end

function deviation = fractionalDeviation(reference, candidate)
deviation = abs(reference - candidate) / max(abs(reference), 1);
end

function value = finiteOrZero(value)
if ~isscalar(value) || ~isfinite(value)
    value = 0;
end
end

function writeReport(path, audit, local, baselines)
fid = fopen(path, 'w');
if fid < 0
    error('Could not open spliced-cycle audit report.');
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# Backbone residual spliced-cycle audit\n\n');
fprintf(fid, '- Protocol: `%s`\n', audit.protocolId);
fprintf(fid, '- Gate passed: `%d`\n', ...
    audit.headroomGatePassed);
fprintf(fid, '- Constraint eligible: `%d`\n', ...
    audit.constraintEligible);
fprintf(fid, '- Evidence boundary: %s\n\n', ...
    audit.evidenceBoundary);
fprintf(fid, '## Results\n\n');
fprintf(fid, '| Arm | E-OSPA | Worst |\n');
fprintf(fid, '|---|---:|---:|\n');
fprintf(fid, '| `local` | %.4f | %.4f |\n', ...
    local.focusEospa, local.focusWorstSensorEospa);
for idx = 1:numel(baselines)
    fprintf(fid, '| `%s` | %.4f | %.4f |\n', ...
        baselines(idx).mode, ...
        baselines(idx).focusEospa, ...
        baselines(idx).focusWorstSensorEospa);
end
fprintf(fid, '| `%s` | %.4f | %.4f |\n\n', ...
    audit.candidateMode, audit.candidateEospa, ...
    audit.candidateWorstSensorEospa);
fprintf(fid, '- Minimum baseline mean gain: `%.4f%%`\n', ...
    100 * audit.minimumBaselineMeanGainFraction);
fprintf(fid, '- Minimum baseline worst-node gain: `%.4f%%`\n', ...
    100 * audit.minimumBaselineWorstNodeGainFraction);
fprintf(fid, '- Attempted-byte deviation: `%.4f%%`\n', ...
    100 * audit.attemptedByteDeviationFraction);
fprintf(fid, '- Independent sensor B3: `%s`\n', ...
    mat2str(audit.sensorB3Pass));
fprintf(fid, '- Independent formation B3: `%s`\n', ...
    mat2str(audit.formationB3Pass));
fprintf(fid, '- Repair / emergency / infeasible: `%.4g / %.4g / %.4g`\n', ...
    audit.repairRate, audit.payloadEmergencyRate, ...
    audit.topologyInfeasibleRate);
fprintf(fid, '- Mean cross-formation edges: `%.4f` (expected `%d`)\n', ...
    audit.meanCrossFormationEdgeCount, ...
    audit.expectedCrossFormationEdgeCount);
fprintf(fid, '- Mean / tail / bytes / safety / provenance / cross gates: ');
fprintf(fid, '`%d / %d / %d / %d / %d / %d`\n', ...
    audit.gates.mean, audit.gates.tail, ...
    audit.gates.bytes, audit.gates.safety, ...
    audit.gates.provenance, audit.gates.crossCount);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
