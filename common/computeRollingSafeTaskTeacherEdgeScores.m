function [scores, details] = ...
    computeRollingSafeTaskTeacherEdgeScores( ...
        context, receiverIndices, senderIndices, ...
        baselineSources, options)
% COMPUTEROLLINGSAFETASKTEACHEREDGESCORES Privileged edge-value labels.
%
% For every physically available cross-formation replacement, this helper
% compares its labelled expected tracking-risk gain with the same
% receiver's registered intra-formation backbone source.  The residual is
% the correct additive objective for a routing projection: an unselected
% receiver keeps the backbone source, while a selected edge replaces it.
%
% Ground truth is read by selectDirectedTaskRoutingTeacher through
% evaluateLmbTopologyTaskRisk.  These scores are therefore an offline
% attainability diagnostic and must never be described as deployable.

if nargin < 5 || isempty(options)
    options = struct();
end
sourceWeight = getField(options, 'sourceWeight', 0.70);
if ~isscalar(sourceWeight) || ~isfinite(sourceWeight) || ...
        sourceWeight <= 0 || sourceWeight >= 1
    error('Teacher sourceWeight must lie strictly inside (0,1).');
end
receiverIndices = reshape(receiverIndices, [], 1);
senderIndices = reshape(senderIndices, [], 1);
baselineSources = reshape(baselineSources, 1, []);
if numel(receiverIndices) ~= numel(senderIndices)
    error('Teacher edge indices must have the same length.');
end
nodeCount = numel(context.localPosteriorBySensor);
if numel(baselineSources) ~= nodeCount || ...
        any(~isfinite(baselineSources)) || ...
        any(baselineSources < 1 | baselineSources > nodeCount | ...
            mod(baselineSources, 1) ~= 0)
    error('Teacher needs one valid backbone source per receiver.');
end
if any(receiverIndices < 1 | receiverIndices > nodeCount | ...
        senderIndices < 1 | senderIndices > nodeCount | ...
        mod(receiverIndices, 1) ~= 0 | ...
        mod(senderIndices, 1) ~= 0)
    error('Teacher edge indices are invalid.');
end

horizonSteps = reshape(getField( ...
    options, 'horizonSteps', 0), 1, []);
horizonSteps = unique(max(0, round(horizonSteps)));
effectiveHorizonSteps = resolveEffectiveHorizons( ...
    context.model, context.currentTime, horizonSteps);
discountFactor = min(max(getField( ...
    options, 'discountFactor', 0.9), 0), 1);
riskOptions = getField(options, 'riskOptions', struct());
riskOptions.horizonSteps = horizonSteps;
riskOptions.discountFactor = discountFactor;
teacherOptions = struct( ...
    'maxMessagesPerReceiver', 1, ...
    'sourceWeightGrid', sourceWeight, ...
    'minimumRelativeNodeGain', 0, ...
    'deliveryExpectationMode', 'bernoulli-risk', ...
    'riskOptions', riskOptions);
[~, teacherDetails] = ...
    selectDirectedTaskRoutingTeacher(context, teacherOptions);
weightGrid = reshape(teacherDetails.sourceWeightGrid, 1, []);
[weightDifference, weightIdx] = ...
    min(abs(weightGrid - sourceWeight));
if isempty(weightIdx) || weightDifference > 1e-12
    error('Task teacher did not evaluate the requested source weight.');
end

gainByEdge = teacherDetails.firstStepExpectedGainByWeight;
baselineGain = nan(1, nodeCount);
for receiverIdx = 1:nodeCount
    baselineGain(receiverIdx) = gainByEdge( ...
        receiverIdx, baselineSources(receiverIdx), weightIdx);
end
scores = -inf(numel(receiverIndices), 1);
candidateGain = nan(numel(receiverIndices), 1);
candidateRisk = nan(numel(receiverIndices), 1);
unweightedResidual = -inf(numel(receiverIndices), 1);
for exampleIdx = 1:numel(receiverIndices)
    receiverIdx = receiverIndices(exampleIdx);
    senderIdx = senderIndices(exampleIdx);
    candidateGain(exampleIdx) = ...
        gainByEdge(receiverIdx, senderIdx, weightIdx);
    if isfinite(candidateGain(exampleIdx))
        candidateRisk(exampleIdx) = ...
            teacherDetails.nodeRiskBefore(receiverIdx) - ...
            candidateGain(exampleIdx);
    end
    if isfinite(candidateGain(exampleIdx)) && ...
            isfinite(baselineGain(receiverIdx))
        unweightedResidual(exampleIdx) = ...
            candidateGain(exampleIdx) - ...
            baselineGain(receiverIdx);
    end
end
baselineExpectedRisk = ...
    teacherDetails.nodeRiskBefore - baselineGain;
receiverRiskPriority = descendingPercentile( ...
    baselineExpectedRisk);
priorityCoefficient = max(0, getField(options, ...
    'receiverRiskPriorityCoefficient', 0));
scoreMultipliers = 1 + ...
    priorityCoefficient * receiverRiskPriority;
scores = unweightedResidual;
for exampleIdx = 1:numel(receiverIndices)
    receiverIdx = receiverIndices(exampleIdx);
    scores(exampleIdx) = scores(exampleIdx) * ...
        scoreMultipliers(receiverIdx);
end

details = struct();
details.sourceWeight = sourceWeight;
details.horizonSteps = effectiveHorizonSteps;
details.requestedHorizonSteps = horizonSteps;
details.effectiveHorizonSteps = effectiveHorizonSteps;
details.discountFactor = discountFactor;
details.receiverIndices = receiverIndices;
details.senderIndices = senderIndices;
details.candidateExpectedGain = candidateGain;
details.candidateExpectedRisk = candidateRisk;
details.baselineExpectedGainByReceiver = baselineGain;
details.baselineExpectedRiskByReceiver = baselineExpectedRisk;
details.receiverRiskPriority = receiverRiskPriority;
details.receiverRiskPriorityCoefficient = priorityCoefficient;
details.scoreMultipliersByReceiver = scoreMultipliers;
details.unweightedResidualScores = unweightedResidual;
details.residualScores = scores;
details.truthUsed = true;
details.posteriorUsed = true;
details.currentLinkReliabilityUsed = true;
details.teacherDetails = teacherDetails;
end

function percentile = descendingPercentile(values)
values = reshape(values, 1, []);
percentile = zeros(size(values));
finiteIndices = find(isfinite(values));
if isempty(finiteIndices)
    return;
end
[~, order] = sort(values(finiteIndices), 'ascend');
count = numel(finiteIndices);
if count == 1
    percentile(finiteIndices) = 1;
    return;
end
ascendingPercentile = (0:(count - 1)) / (count - 1);
percentile(finiteIndices(order)) = ascendingPercentile;
end

function effective = resolveEffectiveHorizons( ...
        model, currentTime, requested)
effective = requested;
if ~isfield(model, 'dynamicTopologyScenario') || ...
        ~isfield(model.dynamicTopologyScenario, ...
            'targetTrajectories') || ...
        isempty(model.dynamicTopologyScenario.targetTrajectories)
    return;
end
trajectories = model.dynamicTopologyScenario.targetTrajectories;
if ~iscell(trajectories)
    trajectories = {trajectories};
end
timeCounts = cellfun(@(trajectory) ...
    size(trajectory, 2), trajectories);
timeCount = min(timeCounts);
effective = requested(currentTime + requested <= timeCount);
if isempty(effective)
    effective = 0;
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
