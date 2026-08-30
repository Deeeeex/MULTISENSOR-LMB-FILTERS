function bank = enumerateOutputAlignedSafeGraphNeighborhoodV155( ...
        context, options)
% ENUMERATEOUTPUTALIGNEDSAFEGRAPHNEIGHBORHOODV155 Canonical radius-one bank.

if nargin < 2 || isempty(options)
    options = struct();
end
protocol = getField(options, 'protocol', ...
    getOutputAlignedSafeGraphNeighborhoodV155Protocol());
nodeCount = numel(context.localPosteriorBySensor);
presetIdx = find(protocol.nodeCounts == nodeCount);
if numel(presetIdx) ~= 1
    error('V155 graph-neighborhood scale is outside the protocol.');
end
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
if numel(groupIds) ~= nodeCount || ...
        numel(unique(groupIds, 'stable')) ~= ...
            protocol.formationCounts(presetIdx)
    error('V155 graph-neighborhood formation contract changed.');
end

[referenceAdjacency, referenceDetails] = ...
    selectBackboneResidualSplicedCyclePolicy( ...
        context, protocol.fallbackMode, struct( ...
            'dominantWeight', protocol.dominantWeight, ...
            'residualWeight', protocol.residualWeight));
[receiverIndices, senderIndices, candidates] = ...
    enumerateBackboneResidualSpliceCandidates( ...
        context, struct( ...
            'dominantWeight', protocol.dominantWeight, ...
            'residualWeight', protocol.residualWeight));
scoreMatrix = -inf(nodeCount);
for edgeIdx = 1:numel(receiverIndices)
    scoreMatrix(receiverIndices(edgeIdx), senderIndices(edgeIdx)) = 0;
end
referenceCuts = reshape( ...
    referenceDetails.spliceSelection.cutReceivers, 1, []);
maximumCount = protocol.candidateCounts(presetIdx) + 1;
selections = enumerateTopKSplicedResidualFormationCycles( ...
    groupIds, candidates.residualBaselineSourcesByReceiver, ...
    candidates.dominantSourcesByReceiver, ...
    candidates.physicalAdjacency, scoreMatrix, ...
    protocol.orientation, maximumCount, referenceCuts, ...
    protocol.maximumCutChanges);

changeCounts = [selections.referenceCutChangeCount];
selections = selections(changeCounts == 1);
if numel(selections) ~= protocol.candidateCounts(presetIdx)
    error('V155 graph-neighborhood did not produce the frozen count.');
end
changedPositions = zeros(1, numel(selections));
changedReceivers = zeros(1, numel(selections));
for candidateIdx = 1:numel(selections)
    changed = find(selections(candidateIdx).cutReceivers ~= ...
        referenceCuts);
    if numel(changed) ~= 1
        error('V155 radius-one candidate changed the wrong cut count.');
    end
    changedPositions(candidateIdx) = changed;
    changedReceivers(candidateIdx) = ...
        selections(candidateIdx).cutReceivers(changed);
end
ranking = [reshape(changedPositions, [], 1), ...
    reshape(changedReceivers, [], 1)];
[~, order] = sortrows(ranking, [1, 2]);
selections = selections(order);
changedPositions = changedPositions(order);
changedReceivers = changedReceivers(order);

candidateAdjacency = false(nodeCount, nodeCount, numel(selections));
candidateFusionWeights = zeros( ...
    nodeCount, nodeCount, numel(selections));
candidateDetails = cell(1, numel(selections));
keys = cell(1, numel(selections));
for candidateIdx = 1:numel(selections)
    selection = selections(candidateIdx);
    [adjacency, fusionWeights, routeDetails] = ...
        buildBackbonePreservingResidualRoute( ...
            context, candidates.dominantAdjacency, ...
            selection.residualAdjacency, ...
            protocol.dominantWeight, protocol.residualWeight);
    if nnz(adjacency) ~= ...
            protocol.exactSelectedMessageCount(presetIdx) || ...
            any(adjacency(:) & ...
                ~logical(context.physicalAdjacency(:)))
        error('V155 candidate violates message or physical constraints.');
    end
    details = routeDetails;
    details.mode = 'output-aligned-safe-graph-neighborhood-v155';
    details.objective = NaN;
    details.candidateIndex = candidateIdx;
    details.selectionSeconds = 0;
    details.taskRisk = NaN;
    details.baselineTaskRisk = NaN;
    details.taskAdvantage = NaN;
    details.taskRiskSpread = NaN;
    details.validCandidateCount = numel(selections);
    details.fusionWeightMatrix = fusionWeights;
    details.baselineAdjacency = referenceAdjacency;
    details.dominantAdjacency = candidates.dominantAdjacency;
    details.residualAdjacency = selection.residualAdjacency;
    details.residualBaselineAdjacency = ...
        candidates.residualBaselineAdjacency;
    details.spliceSelection = selection;
    details.selectedSourcesByReceiver = ...
        selection.residualSourcesByReceiver;
    details.overrideMask = ...
        selection.residualSourcesByReceiver ~= ...
            candidates.residualBaselineSourcesByReceiver;
    details.overrideFraction = mean(details.overrideMask);
    details.crossFormationMessageCount = ...
        selection.crossFormationEdgeCount;
    details.maximumCrossEdges = selection.crossFormationEdgeCount;
    details.maximumCrossSourceLoad = ...
        selection.maximumCrossSourceLoad;
    details.maximumCrossReceiverLoad = ...
        selection.maximumCrossReceiverLoad;
    details.proposalCrossCount = selection.crossFormationEdgeCount;
    details.repairTriggered = false;
    details.payloadConstraintEnforced = false;
    details.payloadLimitPassed = NaN;
    details.payloadEmergencyUsed = false;
    details.posteriorUsed = false;
    details.posteriorPayloadMetadataUsed = false;
    details.currentLinkReliabilityUsed = false;
    details.currentPhysicalActionSetUsed = true;
    details.backboneMode = ...
        'fixed-index-plus-radius-one-spliced-residual-cycle';
    details.sourceWeight = protocol.residualWeight;
    details.sensorWindowMature = true;
    details.sensorWindowStrongConnected = true;
    details.formationWindowMature = true;
    details.formationWindowStrongConnected = true;
    details.successorSensorStrongConnected = NaN;
    details.oneStepTopologyReserveChecked = false;
    details.oneStepTopologyReservePassed = NaN;
    details.oneStepJointProjectionUsed = false;
    details.recursiveSafetyClaimed = false;
    details.topologyInfeasible = false;
    details.truthUsed = false;
    details.groundTruthUsed = false;
    details.futureOutcomeUsed = false;
    details.v155ChangedFormationPosition = ...
        changedPositions(candidateIdx);
    details.v155ChangedReceiver = changedReceivers(candidateIdx);
    details.v155ReferenceCuts = referenceCuts;
    details.v155CandidateCuts = selection.cutReceivers;
    candidateAdjacency(:, :, candidateIdx) = adjacency;
    candidateFusionWeights(:, :, candidateIdx) = fusionWeights;
    candidateDetails{candidateIdx} = details;
    keys{candidateIdx} = sprintf('%d,', adjacency(:));
end
if numel(unique(keys)) ~= numel(keys)
    error('V155 graph-neighborhood contains duplicate complete graphs.');
end

bank = struct();
bank.contractVersion = 'safe-graph-neighborhood-bank-v155-v1';
bank.protocolId = protocol.id;
bank.nodeCount = nodeCount;
bank.formationCount = protocol.formationCounts(presetIdx);
bank.candidateCount = numel(selections);
bank.referenceAdjacency = referenceAdjacency;
bank.referenceFusionWeights = referenceDetails.fusionWeightMatrix;
bank.referenceDetails = referenceDetails;
bank.referenceCuts = referenceCuts;
bank.candidateAdjacency = candidateAdjacency;
bank.candidateFusionWeights = candidateFusionWeights;
bank.candidateDetails = candidateDetails;
bank.changedFormationPositions = changedPositions;
bank.changedReceivers = changedReceivers;
bank.allCandidatesDistinct = true;
bank.allCandidatesTruthFree = true;
bank.allCandidatesFutureFree = true;
bank.truthUsed = false;
bank.futureOutcomeUsed = false;
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
