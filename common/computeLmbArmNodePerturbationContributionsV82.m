function result = computeLmbArmNodePerturbationContributionsV82( ...
        referencePosteriors, candidatePosteriors, model, options)
% COMPUTELMBARMNODEPERTURBATIONCONTRIBUTIONSV82 Exact node split of V77.

if nargin < 4 || isempty(options)
    options = struct();
end
nodeCount = numel(referencePosteriors);
regularization = getField(options, ...
    'covarianceRegularization', 1e-9);
if ~iscell(referencePosteriors) || ...
        ~iscell(candidatePosteriors) || nodeCount < 2 || ...
        numel(candidatePosteriors) ~= nodeCount || ...
        ~isscalar(regularization) || ~isfinite(regularization) || ...
        regularization < 0
    error('NodePerturbationV82:InvalidInput', ...
        'Arm posterior collections or options are invalid.');
end

reference = cell(1, nodeCount);
candidate = cell(1, nodeCount);
labels = zeros(2, 0);
for nodeIdx = 1:nodeCount
    reference{nodeIdx} = summarizeLmbPosteriorForDisagreement( ...
        referencePosteriors{nodeIdx}, model);
    candidate{nodeIdx} = summarizeLmbPosteriorForDisagreement( ...
        candidatePosteriors{nodeIdx}, model);
    labels = unionLabels(labels, reference{nodeIdx}.labels);
    labels = unionLabels(labels, candidate{nodeIdx}.labels);
end

labelRecords = repmat(emptyLabelRecord(nodeCount), 1, 0);
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    referenceExistence = zeros(1, nodeCount);
    candidateExistence = zeros(1, nodeCount);
    positionDelta = zeros(2, nodeCount);
    covarianceNumerator = zeros(2);
    covarianceWeight = 0;
    for nodeIdx = 1:nodeCount
        referenceIdx = findLabel(reference{nodeIdx}.labels, label);
        candidateIdx = findLabel(candidate{nodeIdx}.labels, label);
        referenceExistence(nodeIdx) = valueOrZero( ...
            reference{nodeIdx}.existence, referenceIdx);
        candidateExistence(nodeIdx) = valueOrZero( ...
            candidate{nodeIdx}.existence, candidateIdx);
        if referenceIdx > 0 && candidateIdx > 0
            support = min(referenceExistence(nodeIdx), ...
                candidateExistence(nodeIdx));
            if support > eps
                positionDelta(:, nodeIdx) = sqrt(support) * ( ...
                    candidate{nodeIdx}.positionMean(:, candidateIdx) - ...
                    reference{nodeIdx}.positionMean(:, referenceIdx));
                covarianceNumerator = covarianceNumerator + support * ( ...
                    reference{nodeIdx}.positionCovariance(:, :, referenceIdx) + ...
                    candidate{nodeIdx}.positionCovariance(:, :, candidateIdx));
                covarianceWeight = covarianceWeight + support;
            end
        end
    end
    salience = mean(max(referenceExistence, candidateExistence));
    if salience <= eps
        continue;
    end
    existenceDelta = candidateExistence - referenceExistence;
    existenceCentered = existenceDelta - mean(existenceDelta);
    existenceContribution = existenceCentered .^ 2 / nodeCount;
    spatialContribution = zeros(1, nodeCount);
    if covarianceWeight > eps
        covariance = covarianceNumerator / covarianceWeight;
        covariance = (covariance + covariance') / 2 + ...
            regularization * eye(2);
        centeredPosition = positionDelta - mean(positionDelta, 2);
        for nodeIdx = 1:nodeCount
            spatialContribution(nodeIdx) = quadraticEnergy( ...
                centeredPosition(:, nodeIdx), covariance) / nodeCount;
        end
    end
    record = emptyLabelRecord(nodeCount);
    record.label = label;
    record.salience = salience;
    record.existenceContributionByNode = existenceContribution;
    record.spatialContributionByNode = spatialContribution;
    record.totalContributionByNode = ...
        existenceContribution + spatialContribution;
    labelRecords(end + 1) = record; %#ok<AGROW>
end

existenceByNode = zeros(1, nodeCount);
spatialByNode = zeros(1, nodeCount);
weights = [labelRecords.salience];
if ~isempty(weights) && sum(weights) > eps
    weights = weights / sum(weights);
    for labelIdx = 1:numel(labelRecords)
        existenceByNode = existenceByNode + weights(labelIdx) * ...
            labelRecords(labelIdx).existenceContributionByNode;
        spatialByNode = spatialByNode + weights(labelIdx) * ...
            labelRecords(labelIdx).spatialContributionByNode;
    end
end
totalByNode = existenceByNode + spatialByNode;
v77 = computeLmbArmPerturbationModesV77( ...
    referencePosteriors, candidatePosteriors, model, options);
closureError = abs(sum(totalByNode) - ...
    v77.aggregate.centeredEnergy);

result = struct();
result.contractVersion = ...
    'lmb-arm-node-perturbation-contributions-v82-v1';
result.nodeCount = nodeCount;
result.labelRecords = labelRecords;
result.labelWeights = weights;
result.existenceContributionByNode = existenceByNode;
result.spatialContributionByNode = spatialByNode;
result.totalContributionByNode = totalByNode;
result.centeredEnergy = sum(totalByNode);
result.v77CenteredEnergy = v77.aggregate.centeredEnergy;
result.closureError = closureError;
result.nonnegative = all(totalByNode >= -1e-12);
result.truthUsed = false;
result.futureMeasurementUsed = false;
end

function energy = quadraticEnergy(vector, covariance)
energy = vector' * (covariance \ vector);
if ~isfinite(energy) || energy < -1e-10
    error('NodePerturbationV82:InvalidEnergy', ...
        'A node perturbation energy is invalid.');
end
energy = max(energy, 0);
end

function labels = unionLabels(labels, additions)
for labelIdx = 1:size(additions, 2)
    label = additions(:, labelIdx);
    if isempty(findLabel(labels, label))
        labels(:, end + 1) = label; %#ok<AGROW>
    end
end
end

function index = findLabel(labels, label)
index = [];
if ~isempty(labels)
    index = find(all(bsxfun(@eq, labels, label), 1), 1);
end
end

function value = valueOrZero(values, index)
if isempty(index)
    value = 0;
else
    value = values(index);
end
end

function record = emptyLabelRecord(nodeCount)
record = struct( ...
    'label', zeros(2, 1), ...
    'salience', NaN, ...
    'existenceContributionByNode', zeros(1, nodeCount), ...
    'spatialContributionByNode', zeros(1, nodeCount), ...
    'totalContributionByNode', zeros(1, nodeCount));
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
