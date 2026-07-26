function [adjacency, details] = ...
    selectFormationTreeRoutingTeacher(context, options)
% SELECTFORMATIONTREEROUTINGTEACHER Privileged rooted-tree diagnostic.
%
% The teacher evaluates one-step Bernoulli delivery risk for every physical
% cross-formation route, measures its residual relative to balanced
% round-robin, and projects the residuals onto a rooted formation spanning
% tree. Ground truth is used by the risk labels, so this policy is an
% architecture upper-signal diagnostic and is not deployable.

if nargin < 2 || isempty(options)
    options = struct();
end
timerId = tic;
sourceWeight = getField(options, 'sourceWeight', 0.50);
baselinePhase = max(1, round(getField( ...
    options, 'baselinePhase', 1)));
baselineMode = getField( ...
    options, 'baselineMode', 'round-robin-balanced');
requirePreviousUnionStrongConnectivity = logical(getField( ...
    options, 'requirePreviousUnionStrongConnectivity', false));
if ~isscalar(sourceWeight) || ~isfinite(sourceWeight) || ...
        sourceWeight <= 0 || sourceWeight >= 1
    error('sourceWeight must be strictly between zero and one.');
end

nodeCount = numel(context.localPosteriorBySensor);
groupIds = resolveGroupIds(context.model, nodeCount);
physical = logical(context.physicalAdjacency);
if ~isequal(size(physical), [nodeCount, nodeCount])
    error('physicalAdjacency must be S-by-S.');
end
physical(1:nodeCount+1:end) = false;

[~, baselineDetails] = selectRegisteredDirectedRoutingPolicy( ...
    context, baselineMode, struct( ...
        'sourceWeight', sourceWeight, ...
        'phase', baselinePhase));
baselineSources = ...
    baselineDetails.selectedSourcesByReceiver;
teacherOptions = struct( ...
    'maxMessagesPerReceiver', 1, ...
    'sourceWeightGrid', sourceWeight, ...
    'minimumRelativeNodeGain', 0, ...
    'deliveryExpectationMode', 'bernoulli-risk');
[~, teacherDetails] = ...
    selectDirectedTaskRoutingTeacher(context, teacherOptions);
weightGrid = reshape(teacherDetails.sourceWeightGrid, 1, []);
[difference, weightIdx] = min(abs(weightGrid - sourceWeight));
if difference > 1e-12
    error('Formation-tree teacher did not evaluate the source weight.');
end
expectedGain = teacherDetails.firstStepExpectedGainByWeight;
baselineGain = nan(1, nodeCount);
for receiverIdx = 1:nodeCount
    baselineGain(receiverIdx) = expectedGain( ...
        receiverIdx, baselineSources(receiverIdx), weightIdx);
end

receiverIndices = zeros(0, 1);
senderIndices = zeros(0, 1);
residualScores = zeros(0, 1);
for receiverIdx = 1:nodeCount
    for senderIdx = reshape(find(physical(receiverIdx, :)), 1, [])
        if groupIds(receiverIdx) == groupIds(senderIdx)
            continue;
        end
        candidateGain = expectedGain( ...
            receiverIdx, senderIdx, weightIdx);
        if ~isfinite(candidateGain)
            continue;
        end
        receiverIndices(end + 1, 1) = receiverIdx; %#ok<AGROW>
        senderIndices(end + 1, 1) = senderIdx; %#ok<AGROW>
        residualScores(end + 1, 1) = ...
            candidateGain - baselineGain(receiverIdx); %#ok<AGROW>
    end
end
projectionOptions = struct();
if requirePreviousUnionStrongConnectivity && ...
        isfield(context, 'previousAdjacency') && ...
        isequal(size(context.previousAdjacency), ...
            [nodeCount, nodeCount])
    projectionOptions.requiredUnionFormationAdjacency = ...
        formationAdjacencyFromSensorAdjacency( ...
            context.previousAdjacency, groupIds);
end
selection = selectRootedFormationTreeEdges( ...
    groupIds, receiverIndices, senderIndices, residualScores, ...
    projectionOptions);

selectedSources = baselineSources;
selectedSources(selection.receiverIndices) = ...
    selection.senderIndices;
messageBudget = max(0, floor(getField( ...
    context, 'directedMessageBudget', inf)));
if nodeCount > messageBudget
    error([ ...
        'Formation-tree routing needs one message per receiver; ', ...
        'the directed-message budget is too small.']);
end
adjacency = false(nodeCount);
fusionWeights = eye(nodeCount);
selectedWeights = zeros(1, nodeCount);
for receiverIdx = 1:nodeCount
    senderIdx = selectedSources(receiverIdx);
    if ~physical(receiverIdx, senderIdx)
        error([ ...
            'Formation-tree teacher selected a non-physical route at ', ...
            't=%d: sender %d -> receiver %d.'], ...
            context.currentTime, senderIdx, receiverIdx);
    end
    adjacency(receiverIdx, senderIdx) = true;
    fusionWeights(receiverIdx, receiverIdx) = 1 - sourceWeight;
    fusionWeights(receiverIdx, senderIdx) = sourceWeight;
    selectedWeights(receiverIdx) = sourceWeight;
end

overrideMask = false(1, nodeCount);
overrideMask(selection.receiverIndices) = true;
selectedExampleMask = false(size(residualScores));
selectedExampleMask(selection.exampleIndices) = true;
details = struct();
details.mode = 'oracle-rooted-formation-tree';
details.objective = -sum(residualScores(selectedExampleMask));
details.candidateIndex = NaN;
details.selectionSeconds = toc(timerId);
details.taskRisk = NaN;
details.baselineTaskRisk = NaN;
details.taskAdvantage = sum(residualScores(selectedExampleMask));
details.taskRiskSpread = max(residualScores) - min(residualScores);
details.validCandidateCount = numel(residualScores);
details.directed = true;
details.fusionWeightMatrix = fusionWeights;
details.selectedSourcesByReceiver = selectedSources;
details.selectedSourceWeightsByReceiver = selectedWeights;
details.baselineSourcesByReceiver = baselineSources;
details.overrideMask = overrideMask;
details.overrideFraction = mean(overrideMask);
details.supportedCandidateFraction = 1;
details.crossFormationMessageCount = nnz(overrideMask);
details.messageCount = nnz(adjacency);
details.sourceWeight = sourceWeight;
details.baselineMode = baselineMode;
details.requirePreviousUnionStrongConnectivity = ...
    requirePreviousUnionStrongConnectivity;
details.rootFormation = selection.rootFormation;
details.formationParents = selection.formationParents;
details.selectedTreeResiduals = ...
    reshape(residualScores(selection.exampleIndices), 1, []);
details.maximumCrossSourceLoad = selection.maximumSourceLoad;
details.posteriorUsed = true;
details.truthUsed = true;
details.currentLinkReliabilityUsed = true;
details.currentPhysicalActionSetUsed = true;
details.teacherDetails = teacherDetails;
end

function adjacency = formationAdjacencyFromSensorAdjacency( ...
        sensorAdjacency, groupIds)
groups = unique(groupIds, 'stable');
groupCount = numel(groups);
adjacency = false(groupCount);
sensorAdjacency = logical(sensorAdjacency);
for receiverIdx = 1:numel(groupIds)
    receiverGroup = find(groups == groupIds(receiverIdx), 1);
    for senderIdx = reshape(find(sensorAdjacency(receiverIdx, :)), 1, [])
        senderGroup = find(groups == groupIds(senderIdx), 1);
        if receiverGroup ~= senderGroup
            adjacency(senderGroup, receiverGroup) = true;
        end
    end
end
end

function groupIds = resolveGroupIds(model, nodeCount)
if ~isfield(model, 'dynamicTopologyScenario') || ...
        ~isfield(model.dynamicTopologyScenario, 'config') || ...
        ~isfield(model.dynamicTopologyScenario.config, 'sensorGroupIds')
    error('Formation-tree routing requires sensorGroupIds metadata.');
end
groupIds = reshape( ...
    model.dynamicTopologyScenario.config.sensorGroupIds, 1, []);
if numel(groupIds) ~= nodeCount || ...
        any(~isfinite(groupIds)) || any(groupIds < 1) || ...
        any(mod(groupIds, 1) ~= 0)
    error('sensorGroupIds must contain one positive integer per sensor.');
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
