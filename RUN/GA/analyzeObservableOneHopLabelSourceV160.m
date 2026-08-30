function [reportPath, summary] = ...
        analyzeObservableOneHopLabelSourceV160(options)
% ANALYZEOBSERVABLEONEHOPLABELSOURCEV160 Truth-free source rankers.
%
% Conditioned on the 35 V159 high-value receiver-label decisions, rank only
% currently reachable one-hop local posteriors using present-time posterior,
% evidence, FoV and payload features. Truth scores the chosen source offline;
% it is never an input to a ranker. This is a source-selection gate, not a
% deployable label-trigger policy.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getPositiveValueReferenceLabelV157Protocol();
valueThreshold = getField(options, 'valueThreshold', 0.1);
candidateScreenPath = getField(options, 'candidateScreenPath', fullfile( ...
    protocol.headroomOutputRoot, 'x36_t72_h8', 'screen', ...
    ['TRACKING_ALIGNED_X36_SCHEDULE_H8_', ...
     'X36_FORMATION_FOV_SEED211_T72_D8.mat']));
referenceScreenPath = getField(options, 'referenceScreenPath', ...
    protocol.shadowCaptureScreenPath);
selectionPath = getField(options, 'selectionPath', fullfile( ...
    protocol.headroomOutputRoot, 'selection_replay', ...
    'POSITIVE_VALUE_REFERENCE_LABEL_SELECTION_V158.mat'));
sourcePath = getField(options, 'sourcePath', fullfile( ...
    protocol.headroomOutputRoot, 'causal_source_preflight', ...
    'CAUSAL_SOURCE_LABEL_AVAILABILITY_V159.mat'));
outputRoot = getField(options, 'outputRoot', fullfile( ...
    protocol.headroomOutputRoot, 'observable_source_proxy'));
reportPath = fullfile(outputRoot, ...
    'OBSERVABLE_ONE_HOP_LABEL_SOURCE_V160.md');
matPath = fullfile(outputRoot, ...
    'OBSERVABLE_ONE_HOP_LABEL_SOURCE_V160.mat');
requiredPaths = {candidateScreenPath, referenceScreenPath, ...
    selectionPath, sourcePath};
if any(cellfun(@(path) exist(path, 'file') ~= 2, requiredPaths))
    error('ObservableSourceV160:MissingInput', ...
        'V157--V159 artifacts are required.');
end

candidateLoaded = load(candidateScreenPath, 'screen');
referenceLoaded = load(referenceScreenPath, 'screen');
selectionLoaded = load(selectionPath, 'summary');
sourceLoaded = load(sourcePath, 'summary');
candidateScreen = candidateLoaded.screen;
referenceScreen = referenceLoaded.screen;
selection = selectionLoaded.summary;
sourceAudit = sourceLoaded.summary;
candidateOutcome = outcomeByAction(candidateScreen, ...
    protocol.candidateActionName);
referenceOutcome = outcomeByAction(referenceScreen, ...
    'reference-full-payload');
workingPages = candidateOutcome.fusedPosteriorSnapshotsByTime;
localPages = candidateOutcome.localPosteriorSnapshotsByTime;
referencePages = referenceOutcome.fusedPosteriorSnapshotsByTime;

inputs = generateDynamicTopologyScenarioInputs( ...
    candidateScreen.presetName, candidateScreen.seed);
oracleModel = inputs.model;
if ~isfield(oracleModel, 'dynamicTopologyScenario') || ...
        ~isstruct(oracleModel.dynamicTopologyScenario)
    oracleModel.dynamicTopologyScenario = struct();
end
oracleModel.dynamicTopologyScenario.targetTrajectories = ...
    inputs.targetTrajectories;
groupIds = reshape(candidateScreen.sensorGroupIds, 1, []);
nodeCount = numel(groupIds);
positionCutoff = resolvePositionCutoff(oracleModel);
recipeNames = { ...
    'minimum-source-bayes-risk', ...
    'maximum-source-evidence-quality', ...
    'handover-rescue', ...
    'compatibility-gated-rescue', ...
    'handover-rescue-per-kib'};

decisions = repmat(emptyDecision(recipeNames), 1, 0);
candidates = repmat(emptyCandidate(recipeNames), 1, 0);
for selectionRow = selection.rows
    pageIdx = selectionRow.page;
    receiverIdx = selectionRow.sensor;
    currentTime = selectionRow.time;
    working = workingPages{pageIdx}{receiverIdx};
    physical = logical(inputs.graphData. ...
        physicalAdjacency(:, :, currentTime));
    physical = physical | physical';
    physical(1:nodeCount+1:end) = false;
    neighborIds = reshape(find(physical(receiverIdx, :)), 1, []);
    for rankIdx = 1:selectionRow.selectedLabelCount
        label = selectionRow.selectedLabels(:, rankIdx);
        referenceObject = findLabelObject( ...
            referencePages{pageIdx}{receiverIdx}, label);
        referenceTrial = replaceLabelObject(working, referenceObject);
        referenceGain = evaluateRisk(working, oracleModel, currentTime) - ...
            evaluateRisk(referenceTrial, oracleModel, currentTime);
        if referenceGain > valueThreshold
            decision = emptyDecision(recipeNames);
            decision.page = pageIdx;
            decision.time = currentTime;
            decision.formation = groupIds(receiverIdx);
            decision.receiver = receiverIdx;
            decision.rank = rankIdx;
            decision.label = label;
            decision.labelText = sprintf('(%d,%d)', label(1), label(2));
            decision.referenceGain = referenceGain;
            decision.candidateStart = numel(candidates) + 1;
            receiverObject = findLabelObject(working, label);
            currentRisk = evaluateRisk(working, oracleModel, currentTime);
            for sourceIdx = neighborIds
                sourceObject = findLabelObject( ...
                    localPages{pageIdx}{sourceIdx}, label);
                if isempty(sourceObject)
                    continue;
                end
                candidate = buildCandidate( ...
                    receiverObject, sourceObject, receiverIdx, sourceIdx, ...
                    groupIds, currentTime, currentRisk, working, ...
                    oracleModel, positionCutoff, recipeNames);
                candidate.decision = numel(decisions) + 1;
                candidates(end + 1) = candidate; %#ok<AGROW>
            end
            decision.candidateEnd = numel(candidates);
            if decision.candidateEnd < decision.candidateStart
                error('ObservableSourceV160:NoOneHopSource', ...
                    'A V159 high-value label has no one-hop local source.');
            end
            decision.candidateCount = ...
                decision.candidateEnd - decision.candidateStart + 1;
            localRows = candidates( ...
                decision.candidateStart:decision.candidateEnd);
            [~, truthBestIdx] = max([localRows.truthGain]);
            decision.truthBestSource = localRows(truthBestIdx).source;
            decision.truthBestGain = localRows(truthBestIdx).truthGain;
            for recipeIdx = 1:numel(recipeNames)
                scores = arrayfun(@(item) ...
                    item.recipeScores(recipeIdx), localRows);
                selectedIdx = stableMaximum(scores, [localRows.source]);
                selected = localRows(selectedIdx);
                decision.selectedSource(recipeIdx) = selected.source;
                decision.selectedTruthGain(recipeIdx) = ...
                    selected.truthGain;
                decision.selectedScore(recipeIdx) = ...
                    selected.recipeScores(recipeIdx);
                decision.exactTruthBest(recipeIdx) = ...
                    selected.source == decision.truthBestSource;
            end
            decisions(end + 1) = decision; %#ok<AGROW>
        end
        working = referenceTrial;
    end
end
if numel(decisions) ~= sourceAudit.highValueEditCount
    error('ObservableSourceV160:DecisionCountDrift', ...
        'The high-value V159 decision count did not replay.');
end

referenceGains = [decisions.referenceGain];
recipeMetrics = repmat(emptyRecipeMetric(), 1, numel(recipeNames));
for recipeIdx = 1:numel(recipeNames)
    selectedGains = arrayfun(@(item) ...
        item.selectedTruthGain(recipeIdx), decisions);
    metric = emptyRecipeMetric();
    metric.name = recipeNames{recipeIdx};
    metric.positiveCount = nnz(selectedGains > 1e-9);
    metric.halfValueCount = nnz( ...
        selectedGains >= 0.5 * referenceGains);
    metric.exactTruthBestCount = nnz(arrayfun(@(item) ...
        item.exactTruthBest(recipeIdx), decisions));
    metric.cappedGainCoveragePercent = gainCoverage( ...
        selectedGains, referenceGains);
    metric.uncappedGainPercent = 100 * sum(selectedGains) / ...
        max(sum(referenceGains), eps);
    metric.minimumRelativeValuePercent = min( ...
        100 * selectedGains ./ max(referenceGains, eps));
    recipeMetrics(recipeIdx) = metric;
end
selectedRecipeIdx = selectSafetyFirstRecipe(recipeMetrics);

summary = struct();
summary.contractVersion = ...
    'observable-one-hop-label-source-v160-v1';
summary.presetName = candidateScreen.presetName;
summary.seed = candidateScreen.seed;
summary.valueThreshold = valueThreshold;
summary.decisionCount = numel(decisions);
summary.candidateCount = numel(candidates);
summary.receiverMissingLabelCount = nnz(arrayfun(@(item) ...
    item.receiverMissing, candidates));
summary.recipeNames = recipeNames;
summary.recipeMetrics = recipeMetrics;
summary.selectedDevelopmentRecipe = recipeNames{selectedRecipeIdx};
summary.selectedDevelopmentRecipeIndex = selectedRecipeIdx;
summary.decisions = decisions;
summary.candidates = candidates;
summary.truthUsedForScoringOnly = true;
summary.truthUsedByRankers = false;
summary.futureInformationUsedByRankers = false;
summary.deployable = false;
summary.evidenceBoundary = [ ...
    'V160 is conditioned on the privileged V159 set of 35 high-value ', ...
    'receiver-label decisions. It tests only whether present-time ', ...
    'posterior, evidence, FoV and payload summaries can rank current ', ...
    'one-hop local sources. Truth evaluates each source offline and ', ...
    'chooses the development recipe after the fact; it does not enter ', ...
    'any recipe score. V160 does not decide which label to send, does ', ...
    'not charge synopsis bytes, and is not a recursive tracking result.'];

if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
save('-mat7-binary', matPath, 'summary');
writeReport(reportPath, summary);
fprintf('V160 observable one-hop source gate: %s\n', reportPath);
end

function candidate = buildCandidate( ...
        receiverObject, sourceObject, receiverIdx, sourceIdx, groupIds, ...
        currentTime, currentRisk, working, model, positionCutoff, ...
        recipeNames)
candidate = emptyCandidate(recipeNames);
candidate.time = currentTime;
candidate.receiver = receiverIdx;
candidate.source = sourceIdx;
candidate.receiverFormation = groupIds(receiverIdx);
candidate.sourceFormation = groupIds(sourceIdx);
candidate.crossFormation = ...
    groupIds(receiverIdx) ~= groupIds(sourceIdx);
receiver = summarizeObject(receiverObject, positionCutoff);
source = summarizeObject(sourceObject, positionCutoff);
candidate.receiverMissing = ~receiver.present;
candidate.receiverExistence = receiver.existence;
candidate.sourceExistence = source.existence;
candidate.receiverRisk = receiver.bayesRisk;
candidate.sourceRisk = source.bayesRisk;
candidate.receiverPositionTrace = receiver.positionTrace;
candidate.sourcePositionTrace = source.positionTrace;
candidate.receiverEvidence = receiver.evidenceQuality;
candidate.sourceEvidence = source.evidenceQuality;
candidate.receiverOpportunity = observationOpportunity( ...
    model, receiverIdx, receiverObject, currentTime);
candidate.sourceOpportunity = observationOpportunity( ...
    model, sourceIdx, sourceObject, currentTime);
[candidate.mahalanobis, candidate.compatibility] = ...
    compareObjects(receiver, source);
candidate.payloadBytes = labelPayloadBytes(sourceObject);
candidate.truthGain = currentRisk - evaluateRisk( ...
    replaceLabelObject(working, sourceObject), model, currentTime);

existenceGain = max(source.existence - receiver.existence, 0);
precisionGain = max(log((receiver.positionTrace + eps) / ...
    (source.positionTrace + eps)), 0);
precisionGain = 1 - exp(-min(precisionGain, 20));
evidenceGap = max(source.evidenceQuality - ...
    receiver.evidenceQuality, 0);
opportunityGap = max(candidate.sourceOpportunity - ...
    candidate.receiverOpportunity, 0);
disagreement = 1 - exp(-0.5 * min(candidate.mahalanobis, 100));
sourcePrecisionQuality = 1 / (1 + ...
    source.positionTrace / max(positionCutoff^2, eps));
sourceQuality = source.existence * ( ...
    0.40 * source.evidenceQuality + ...
    0.40 * candidate.sourceOpportunity + ...
    0.20 * sourcePrecisionQuality);
riskDominance = receiver.bayesRisk - source.bayesRisk;
rescue = 0.30 * existenceGain + ...
    0.20 * precisionGain + ...
    0.20 * evidenceGap + ...
    0.15 * opportunityGap + ...
    0.15 * sourceQuality * disagreement;
candidate.recipeScores = [ ...
    -source.bayesRisk, ...
    sourceQuality, ...
    rescue + 0.25 * max(riskDominance, 0), ...
    (rescue + 0.25 * max(riskDominance, 0)) * ...
        sqrt(max(candidate.compatibility, 1e-6)), ...
    (rescue + 0.25 * max(riskDominance, 0)) / ...
        max(candidate.payloadBytes / 1024, 0.125)];
end

function summary = summarizeObject(object, positionCutoff)
summary = struct( ...
    'present', false, ...
    'existence', 0, ...
    'mean', zeros(0, 1), ...
    'covariance', zeros(0), ...
    'positionTrace', positionCutoff^2, ...
    'associationConfidence', 0, ...
    'detectionAssociationMass', 0, ...
    'evidenceQuality', 0, ...
    'bayesRisk', 0.5);
if isempty(object) || object.numberOfGmComponents <= 0
    return;
end
[meanVector, covariance] = momentMatch(object);
positionDimension = min(2, numel(meanVector));
positionTrace = max(trace(covariance( ...
    1:positionDimension, 1:positionDimension)), 0);
existence = clamp01(object.r);
association = clamp01(getScalarField( ...
    object, 'associationConfidence', 0));
detection = clamp01(getScalarField( ...
    object, 'detectionAssociationMass', 0));
existenceRisk = min(existence, 1 - existence);
localizationRisk = existence * min( ...
    positionTrace / max(positionCutoff^2, eps), 1);
summary.present = true;
summary.existence = existence;
summary.mean = meanVector;
summary.covariance = covariance;
summary.positionTrace = positionTrace;
summary.associationConfidence = association;
summary.detectionAssociationMass = detection;
summary.evidenceQuality = 0.5 * association + 0.5 * detection;
summary.bayesRisk = 0.5 * existenceRisk + 0.5 * localizationRisk;
end

function [mahalanobis, compatibility] = compareObjects(receiver, source)
if ~receiver.present || ~source.present
    mahalanobis = 0;
    compatibility = 0;
    return;
end
dimension = min(numel(receiver.mean), numel(source.mean));
delta = receiver.mean(1:dimension) - source.mean(1:dimension);
covariance = receiver.covariance(1:dimension, 1:dimension) + ...
    source.covariance(1:dimension, 1:dimension);
covariance = regularizeCovariance(covariance);
mahalanobis = max(real(delta' * (covariance \ delta)), 0);
if ~isfinite(mahalanobis)
    mahalanobis = 1e6;
end
compatibility = exp(-0.5 * min(mahalanobis, 100));
end

function value = observationOpportunity(model, sensorIdx, object, currentTime)
if isempty(object) || object.numberOfGmComponents <= 0
    value = 0;
    return;
end
opportunity = computeLmbLabelObservationOpportunity( ...
    model, sensorIdx, object, currentTime);
value = clamp01(opportunity.expectedDetectionProbability);
end

function [meanVector, covariance] = momentMatch(object)
componentCount = object.numberOfGmComponents;
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if numel(weights) ~= componentCount || sum(weights) <= 0
    weights = ones(1, componentCount);
end
weights = weights / sum(weights);
stateDimension = numel(object.mu{1});
meanVector = zeros(stateDimension, 1);
for componentIdx = 1:componentCount
    meanVector = meanVector + weights(componentIdx) * ...
        object.mu{componentIdx};
end
covariance = zeros(stateDimension);
for componentIdx = 1:componentCount
    delta = object.mu{componentIdx} - meanVector;
    covariance = covariance + weights(componentIdx) * ...
        (object.Sigma{componentIdx} + delta * delta');
end
covariance = regularizeCovariance(covariance);
end

function covariance = regularizeCovariance(covariance)
covariance = (covariance + covariance') / 2;
jitter = 0;
for attempt = 1:8
    [~, flag] = chol(covariance + jitter * eye(size(covariance)));
    if flag == 0
        covariance = covariance + jitter * eye(size(covariance));
        return;
    end
    jitter = max(1e-12, 10 * max(jitter, 1e-12));
end
error('ObservableSourceV160:InvalidCovariance', ...
    'A label covariance cannot be regularized.');
end

function risk = evaluateRisk(posterior, model, currentTime)
[risk, ~] = evaluateLmbTopologyCurrentEospa( ...
    posterior, model, currentTime, struct());
end

function posterior = replaceLabelObject(posterior, object)
posterior = reshape(posterior, 1, []);
label = [object.birthTime; object.birthLocation];
idx = findLabelIndex(posterior, label);
if idx == 0
    posterior(end + 1) = object;
else
    posterior(idx) = object;
end
end

function object = findLabelObject(objects, label)
idx = findLabelIndex(objects, label);
if idx == 0
    object = [];
else
    object = objects(idx);
end
end

function idx = findLabelIndex(objects, label)
idx = 0;
for objectIdx = 1:numel(objects)
    if objects(objectIdx).birthTime == label(1) && ...
            objects(objectIdx).birthLocation == label(2)
        idx = objectIdx;
        return;
    end
end
end

function bytes = labelPayloadBytes(object)
dimension = numel(object.mu{1});
bytes = 8 * (3 + object.numberOfGmComponents * ...
    (1 + dimension + dimension * dimension));
end

function idx = stableMaximum(scores, sourceIds)
maximum = max(scores);
ties = find(abs(scores - maximum) <= 1e-12);
[~, order] = min(sourceIds(ties));
idx = ties(order);
end

function idx = selectSafetyFirstRecipe(metrics)
% Prefer avoiding harmful source choices before maximizing average capture.
keys = [ ...
    [metrics.positiveCount]', ...
    [metrics.halfValueCount]', ...
    [metrics.cappedGainCoveragePercent]', ...
    [metrics.minimumRelativeValuePercent]'];
[~, order] = sortrows(keys, [-1, -2, -3, -4]);
idx = order(1);
end

function value = gainCoverage(gains, referenceGains)
value = 100 * sum(max(min(gains, referenceGains), 0)) / ...
    max(sum(referenceGains), eps);
end

function outcome = outcomeByAction(screen, actionName)
idx = find(strcmp({screen.records.actionName}, actionName), 1);
if isempty(idx)
    error('ObservableSourceV160:MissingAction', ...
        'The requested action is missing: %s', actionName);
end
outcome = screen.outcomes(idx);
end

function value = resolvePositionCutoff(model)
value = NaN;
if isfield(model, 'ospaParameters') && ...
        isstruct(model.ospaParameters) && ...
        isfield(model.ospaParameters, 'eC')
    value = model.ospaParameters.eC;
end
if ~isscalar(value) || ~isfinite(value) || value <= 0
    error('ObservableSourceV160:MissingCutoff', ...
        'A positive E-OSPA position cutoff is required.');
end
end

function writeReport(path, summary)
fileId = fopen(path, 'w');
if fileId < 0
    error('ObservableSourceV160:ReportOpenFailed', ...
        'Could not open the V160 report.');
end
cleanup = onCleanup(@() fclose(fileId)); %#ok<NASGU>
fprintf(fileId, '# V160 observable one-hop label-source gate\n\n');
fprintf(fileId, '- Preset / seed: `%s / %d`\n', ...
    summary.presetName, summary.seed);
fprintf(fileId, '- Conditioned high-value decisions: `%d`\n', ...
    summary.decisionCount);
fprintf(fileId, '- One-hop source-label candidates: `%d`\n', ...
    summary.candidateCount);
fprintf(fileId, '- Receiver-missing candidate rows: `%d`\n\n', ...
    summary.receiverMissingLabelCount);
fprintf(fileId, ['| Observable source recipe | Positive | At least half ', ...
    'reference | Exact truth-best | Capped coverage | Uncapped value | ', ...
    'Minimum relative value |\n']);
fprintf(fileId, '|:--|--:|--:|--:|--:|--:|--:|\n');
for metric = summary.recipeMetrics
    fprintf(fileId, '| %s | %d/%d | %d/%d | %d/%d | %.3f%% | %.3f%% | %.3f%% |\n', ...
        metric.name, metric.positiveCount, summary.decisionCount, ...
        metric.halfValueCount, summary.decisionCount, ...
        metric.exactTruthBestCount, summary.decisionCount, ...
        metric.cappedGainCoveragePercent, metric.uncappedGainPercent, ...
        metric.minimumRelativeValuePercent);
end
fprintf(fileId, '\n- Development-best recipe: `%s`\n\n', ...
    summary.selectedDevelopmentRecipe);
fprintf(fileId, ['| t | F | Receiver | Rank | Label | Ref. gain | ', ...
    'Truth-best source/gain | Development source/gain |\n']);
fprintf(fileId, '|--:|--:|--:|--:|:--|--:|:--|:--|\n');
recipeIdx = summary.selectedDevelopmentRecipeIndex;
for decision = summary.decisions
    fprintf(fileId, '| %d | %d | %d | %d | %s | %.4f | %d/%.4f | %d/%.4f |\n', ...
        decision.time, decision.formation, decision.receiver, ...
        decision.rank, decision.labelText, decision.referenceGain, ...
        decision.truthBestSource, decision.truthBestGain, ...
        decision.selectedSource(recipeIdx), ...
        decision.selectedTruthGain(recipeIdx));
end
fprintf(fileId, '\n## Evidence boundary\n\n%s\n', ...
    summary.evidenceBoundary);
end

function decision = emptyDecision(recipeNames)
recipeCount = numel(recipeNames);
decision = struct( ...
    'page', 0, ...
    'time', 0, ...
    'formation', 0, ...
    'receiver', 0, ...
    'rank', 0, ...
    'label', zeros(2, 1), ...
    'labelText', '', ...
    'referenceGain', 0, ...
    'candidateStart', 0, ...
    'candidateEnd', 0, ...
    'candidateCount', 0, ...
    'truthBestSource', 0, ...
    'truthBestGain', -inf, ...
    'selectedSource', zeros(1, recipeCount), ...
    'selectedTruthGain', -inf(1, recipeCount), ...
    'selectedScore', -inf(1, recipeCount), ...
    'exactTruthBest', false(1, recipeCount));
end

function candidate = emptyCandidate(recipeNames)
candidate = struct( ...
    'decision', 0, ...
    'time', 0, ...
    'receiver', 0, ...
    'source', 0, ...
    'receiverFormation', 0, ...
    'sourceFormation', 0, ...
    'crossFormation', false, ...
    'receiverMissing', false, ...
    'receiverExistence', 0, ...
    'sourceExistence', 0, ...
    'receiverRisk', 0, ...
    'sourceRisk', 0, ...
    'receiverPositionTrace', 0, ...
    'sourcePositionTrace', 0, ...
    'receiverEvidence', 0, ...
    'sourceEvidence', 0, ...
    'receiverOpportunity', 0, ...
    'sourceOpportunity', 0, ...
    'mahalanobis', 0, ...
    'compatibility', 0, ...
    'payloadBytes', 0, ...
    'truthGain', -inf, ...
    'recipeScores', -inf(1, numel(recipeNames)));
end

function metric = emptyRecipeMetric()
metric = struct( ...
    'name', '', ...
    'positiveCount', 0, ...
    'halfValueCount', 0, ...
    'exactTruthBestCount', 0, ...
    'cappedGainCoveragePercent', 0, ...
    'uncappedGainPercent', 0, ...
    'minimumRelativeValuePercent', -inf);
end

function value = getScalarField(data, name, fallback)
value = fallback;
if isstruct(data) && isfield(data, name) && ...
        isscalar(data.(name)) && isfinite(data.(name))
    value = data.(name);
end
end

function value = clamp01(value)
if ~isfinite(value)
    value = 0;
end
value = min(max(value, 0), 1);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
