function [scores, details] = ...
    computeBackbonePreservingResidualTaskTeacherScores( ...
        context, receiverIndices, senderIndices, ...
        residualBaselineSources, options)
% COMPUTEBACKBONEPRESERVINGRESIDUALTASKTEACHERSCORES Privileged marginal.
%
% Each candidate replaces only the low-weight residual sender. The
% high-weight fixed-index sender remains in every candidate. Link delivery
% is marginalized exactly over the one or two distinct attempted sources
% used by that receiver.

if nargin < 5 || isempty(options)
    options = struct();
end
dominantWeight = getField(options, ...
    'dominantWeight', 0.70);
residualWeight = getField(options, ...
    'residualWeight', 0.05);
if dominantWeight <= 0 || residualWeight <= 0 || ...
        dominantWeight + residualWeight >= 1
    error('Residual teacher weights are invalid.');
end
nodeCount = numel(context.localPosteriorBySensor);
receiverIndices = reshape(receiverIndices, [], 1);
senderIndices = reshape(senderIndices, [], 1);
residualBaselineSources = ...
    reshape(residualBaselineSources, 1, []);
if numel(receiverIndices) ~= numel(senderIndices) || ...
        numel(residualBaselineSources) ~= nodeCount
    error('Residual teacher action indices are misaligned.');
end
[~, dominantDetails] = ...
    selectRegisteredDirectedRoutingPolicy( ...
        context, 'fixed-index-star', ...
        struct('sourceWeight', dominantWeight));
dominantSources = reshape( ...
    dominantDetails.selectedSourcesByReceiver, 1, []);
riskOptions = getField(options, ...
    'riskOptions', struct());
riskOptions.horizonSteps = getField( ...
    options, 'horizonSteps', 0);
riskOptions.discountFactor = getField( ...
    options, 'discountFactor', 0.9);

baselineRisk = nan(1, nodeCount);
for receiverIdx = 1:nodeCount
    baselineRisk(receiverIdx) = expectedReceiverRisk( ...
        context, receiverIdx, dominantSources(receiverIdx), ...
        NaN, ...
        dominantWeight, residualWeight, riskOptions);
end
candidateRisk = nan(numel(receiverIndices), 1);
scores = -inf(numel(receiverIndices), 1);
for exampleIdx = 1:numel(receiverIndices)
    receiverIdx = receiverIndices(exampleIdx);
    candidateRisk(exampleIdx) = expectedReceiverRisk( ...
        context, receiverIdx, dominantSources(receiverIdx), ...
        senderIndices(exampleIdx), dominantWeight, ...
        residualWeight, riskOptions);
    if isfinite(candidateRisk(exampleIdx)) && ...
            isfinite(baselineRisk(receiverIdx))
        scores(exampleIdx) = baselineRisk(receiverIdx) - ...
            candidateRisk(exampleIdx);
    end
end

details = struct();
details.contractVersion = ...
    'backbone-preserving-residual-task-teacher-v1';
details.receiverIndices = receiverIndices;
details.senderIndices = senderIndices;
details.dominantSourcesByReceiver = dominantSources;
details.residualBaselineSourcesByReceiver = ...
    residualBaselineSources;
details.dominantWeight = dominantWeight;
details.residualWeight = residualWeight;
details.baselineExpectedRiskByReceiver = baselineRisk;
details.candidateExpectedRisk = candidateRisk;
details.residualScores = scores;
details.truthUsed = true;
details.groundTruthUsed = true;
details.posteriorUsed = true;
details.currentLinkReliabilityUsed = true;
details.futureOutcomeUsed = false;
details.deployable = false;
end

function risk = expectedReceiverRisk( ...
        context, receiverIdx, dominantSource, ...
        residualSource, dominantWeight, ...
        residualWeight, riskOptions)
selfWeight = 1 - dominantWeight - residualWeight;
if isfinite(residualSource)
    sources = [dominantSource, residualSource];
    weights = [dominantWeight, residualWeight];
else
    sources = dominantSource;
    weights = dominantWeight;
    selfWeight = 1 - dominantWeight;
end
if isfinite(residualSource) && dominantSource == residualSource
    uniqueSources = dominantSource;
    uniqueWeights = dominantWeight + residualWeight;
else
    uniqueSources = sources;
    uniqueWeights = weights;
end
reliability = zeros(1, numel(uniqueSources));
for sourceIdx = 1:numel(uniqueSources)
    reliability(sourceIdx) = 1 - edgeDrop( ...
        context.commConfig, uniqueSources(sourceIdx), ...
        receiverIdx, context.currentTime);
end

risk = 0;
for maskCode = 0:(2^numel(uniqueSources) - 1)
    delivered = logical(bitget( ...
        maskCode, 1:numel(uniqueSources)));
    probability = prod( ...
        reliability(delivered)) * prod( ...
        1 - reliability(~delivered));
    if probability <= 0
        continue;
    end
    if ~any(delivered)
        candidatePosterior = ...
            context.localPosteriorBySensor{receiverIdx};
    else
        selectedSources = uniqueSources(delivered);
        fusionInputs = [{ ...
            context.localPosteriorBySensor{receiverIdx}}, ...
            context.localPosteriorBySensor(selectedSources)];
        fusionWeights = [selfWeight, ...
            uniqueWeights(delivered)];
        fusionWeights = fusionWeights / sum(fusionWeights);
        fusionDetails = struct( ...
            'eventType', [0, 2 * ones( ...
                1, numel(selectedSources))]);
        candidatePosterior = fuseLmbPosteriorsByLabel( ...
            fusionInputs, fusionWeights, context.model, ...
            fusionWeights, fusionDetails, ...
            context.triggerConfig);
    end
    risk = risk + probability * ...
        evaluateLmbTopologyTaskRisk( ...
            candidatePosterior, context.model, ...
            context.currentTime, riskOptions);
end
end

function probability = edgeDrop( ...
        config, senderIdx, receiverIdx, currentTime)
if isfield(config, 'pDropByEdge') && ...
        ~isempty(config.pDropByEdge)
    if ndims(config.pDropByEdge) >= 3
        timeIdx = min(currentTime, ...
            size(config.pDropByEdge, 3));
        probability = config.pDropByEdge( ...
            senderIdx, receiverIdx, timeIdx);
    else
        probability = config.pDropByEdge( ...
            senderIdx, receiverIdx);
    end
else
    probability = 0;
end
probability = min(max(probability, 0), 1);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
