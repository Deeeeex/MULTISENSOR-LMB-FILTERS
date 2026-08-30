function [reportPath, result] = ...
        analyzeFormationRepairPropagationTrustV189(options)
% ANALYZEFORMATIONREPAIRPROPAGATIONTRUSTV189 Cross-scale causal preflight.
%
% Replays the already opened V188 proposal pages and asks only whether the
% source and every intended receiver stay within the registered positional
% trust region over the recursive H=3 window.  It reads no truth and does
% not score tracking performance.

if nargin < 1 || isempty(options)
    options = struct();
end
cases = getField(options, 'cases', defaultCases());
horizonSteps = getField(options, 'horizonSteps', 3);
outputRoot = char(getField(options, 'outputRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v189', 'propagation_trust_preflight')));
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end

caseResults = repmat(emptyCaseResult(), 1, numel(cases));
for caseIdx = 1:numel(cases)
    caseResults(caseIdx) = evaluateCase(cases(caseIdx), horizonSteps);
end

result = struct();
result.contractVersion = 'formation-repair-propagation-trust-v189-preflight-v1';
result.generatedAt = datestr(now, 31);
result.horizonSteps = horizonSteps;
result.caseResults = caseResults;
result.truthUsed = false;
result.futureMeasurementsUsed = false;
result.trackingPerformanceScored = false;
result.validationClaimAllowed = false;
result.evidenceBoundary = [ ...
    'This causal replay certifies only open-loop positional compatibility ', ...
    'of already opened V188 proposals. Passing is a necessary safety ', ...
    'condition, not evidence of recursive tracking improvement.'];

matPath = fullfile(outputRoot, ...
    'FORMATION_REPAIR_PROPAGATION_TRUST_V189_PREFLIGHT.mat');
reportPath = fullfile(outputRoot, ...
    'FORMATION_REPAIR_PROPAGATION_TRUST_V189_PREFLIGHT.md');
result.matPath = matPath;
result.reportPath = reportPath;
save('-mat7-binary', matPath, 'result');
writeReport(reportPath, result);
fprintf('V189 propagation-trust preflight: %s\n', reportPath);
end

function caseResult = evaluateCase(caseSpec, horizonSteps)
loadedAnalysis = load(caseSpec.analysisPath, 'analysis');
analysis = loadedAnalysis.analysis;
loadedScreen = load(analysis.screenPath, 'screen');
screen = loadedScreen.screen;
names = {screen.records.actionName};
baseIdx = find(cellfun(@(name) strncmp( ...
    name, caseSpec.baseActionPrefix, ...
    numel(caseSpec.baseActionPrefix)), names), 1);
if isempty(baseIdx)
    error('FormationRepairPropagationTrustV189:MissingBaseArm', ...
        'The causal base arm is absent from %s.', analysis.screenPath);
end
baseOutcome = screen.outcomes(baseIdx);
posterior = baseOutcome.fusedPosteriorSnapshotsByTime{analysis.pageIndex};
inputs = generateDynamicTopologyScenarioInputs( ...
    analysis.presetName, analysis.seed);
model = inputs.model;

proposalResults = repmat(emptyProposalResult(), ...
    1, numel(analysis.proposals));
for proposalIdx = 1:numel(analysis.proposals)
    proposal = analysis.proposals(proposalIdx);
    trust = evaluateFormationRepairPropagationTrustV189( ...
        proposal, posterior, model, ...
        struct('horizonSteps', horizonSteps));
    proposalResult = emptyProposalResult();
    proposalResult.proposalIndex = proposalIdx;
    proposalResult.formationId = proposal.formationId;
    proposalResult.sourceId = proposal.sourceId;
    proposalResult.label = proposal.label;
    proposalResult.selectedByV188 = ismember( ...
        proposalIdx, analysis.projection.selectedProposalIndices);
    proposalResult.passedAllReceivers = trust.passedAllReceivers;
    proposalResult.maximumDistance = trust.maximumDistance;
    proposalResult.maximumNormalizedDistance = ...
        trust.maximumNormalizedDistance;
    proposalResult.maximumDistanceByReceiver = ...
        trust.maximumDistanceByReceiver;
    proposalResult.preflightAttemptedBytes = ...
        trust.preflightAttemptedBytes;
    proposalResult.totalAttemptedBytesIfAccepted = ...
        trust.totalAttemptedBytesIfAccepted;
    proposalResults(proposalIdx) = proposalResult;
end

selectedIndices = analysis.projection.selectedProposalIndices;
caseResult = emptyCaseResult();
caseResult.name = caseSpec.name;
caseResult.presetName = analysis.presetName;
caseResult.seed = analysis.seed;
caseResult.currentTime = analysis.currentTime;
caseResult.positionCutoff = model.ospaParameters.eC;
caseResult.selectedProposalIndices = selectedIndices;
caseResult.proposalResults = proposalResults;
if isempty(selectedIndices)
    caseResult.selectedProposalPassed = false;
    caseResult.selectedProposalMaximumDistance = inf;
else
    selectedResults = proposalResults(selectedIndices);
    caseResult.selectedProposalPassed = ...
        all([selectedResults.passedAllReceivers]);
    caseResult.selectedProposalMaximumDistance = ...
        max([selectedResults.maximumDistance]);
end
end

function writeReport(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('FormationRepairPropagationTrustV189:ReportOpenFailed', ...
        'Cannot write %s.', path);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V189 propagation-trust preflight\n\n');
fprintf(fid, ['This truth-free replay checks whether each already-opened ', ...
    'V188 source and all intended receivers remain within the registered ', ...
    'position cutoff across the H=%d recursive window. It does not score ', ...
    'tracking performance.\n\n'], result.horizonSteps);
for caseIdx = 1:numel(result.caseResults)
    caseResult = result.caseResults(caseIdx);
    fprintf(fid, '## %s\n\n', caseResult.name);
    fprintf(fid, ['- preset `%s`, seed `%d`, t=`%d`, cutoff `%.3f m`\n', ...
        '- V188-selected proposal(s): `%s`; selected trust result: `%s`; ', ...
        'maximum distance `%.3f m`\n\n'], ...
        caseResult.presetName, caseResult.seed, caseResult.currentTime, ...
        caseResult.positionCutoff, ...
        mat2str(caseResult.selectedProposalIndices), ...
        passText(caseResult.selectedProposalPassed), ...
        caseResult.selectedProposalMaximumDistance);
    fprintf(fid, ['| proposal | formation | source | label | V188 selected | ', ...
        'max distance (m) | cutoff ratio | trust | preflight bytes | ', ...
        'accepted total bytes |\n']);
    fprintf(fid, ['|---:|---:|---:|---|---|---:|---:|---|---:|---:|\n']);
    rows = caseResult.proposalResults;
    for rowIdx = 1:numel(rows)
        row = rows(rowIdx);
        fprintf(fid, ...
            '| %d | %d | %d | `[%d,%d]` | %s | %.3f | %.3f | %s | %d | %d |\n', ...
            row.proposalIndex, row.formationId, row.sourceId, ...
            row.label(1), row.label(2), yesNo(row.selectedByV188), ...
            row.maximumDistance, row.maximumNormalizedDistance, ...
            passText(row.passedAllReceivers), ...
            row.preflightAttemptedBytes, ...
            row.totalAttemptedBytesIfAccepted);
    end
    fprintf(fid, '\n');
end
fprintf(fid, '## Finding boundary\n\n%s\n', result.evidenceBoundary);
end

function value = defaultCases()
root = fullfile('RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v188', 'immediate_headroom');
value = repmat(struct( ...
    'name', '', 'analysisPath', '', ...
    'baseActionPrefix', 'v99-online-positive-net-'), 1, 2);
value(1).name = 'M24 seed 211 t=104';
value(1).analysisPath = fullfile(root, ...
    'm24_formation_fov_seed211_t104', ...
    'BUDGET_RECYCLED_REPAIR_V188_M24_FORMATION_FOV_SEED211_T104.mat');
value(2).name = 'X36 seed 211 t=72';
value(2).analysisPath = fullfile(root, ...
    'x36_formation_fov_seed211_t72', ...
    'BUDGET_RECYCLED_REPAIR_V188_X36_FORMATION_FOV_SEED211_T72.mat');
end

function value = emptyProposalResult()
value = struct( ...
    'proposalIndex', 0, 'formationId', 0, 'sourceId', 0, ...
    'label', zeros(2, 1), 'selectedByV188', false, ...
    'passedAllReceivers', false, 'maximumDistance', inf, ...
    'maximumNormalizedDistance', inf, ...
    'maximumDistanceByReceiver', zeros(1, 0), ...
    'preflightAttemptedBytes', 0, ...
    'totalAttemptedBytesIfAccepted', 0);
end

function value = emptyCaseResult()
value = struct( ...
    'name', '', 'presetName', '', 'seed', 0, 'currentTime', 0, ...
    'positionCutoff', NaN, 'selectedProposalIndices', zeros(1, 0), ...
    'proposalResults', repmat(emptyProposalResult(), 1, 0), ...
    'selectedProposalPassed', false, ...
    'selectedProposalMaximumDistance', inf);
end

function value = passText(passed)
if passed
    value = 'pass';
else
    value = 'reject';
end
end

function value = yesNo(flag)
if flag
    value = 'yes';
else
    value = 'no';
end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
