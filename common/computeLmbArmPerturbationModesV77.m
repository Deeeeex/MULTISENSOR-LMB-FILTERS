function result = computeLmbArmPerturbationModesV77( ...
        referencePosteriors, candidatePosteriors, model, options)
% COMPUTELMBARMPERTURBATIONMODESV77 Common/disagreement decomposition.
%
% For each label, the candidate-minus-reference existence and position
% perturbations are decomposed across network nodes into their common mode
% and centered disagreement mode.  A common perturbation can represent
% information that has successfully spread; only centered energy is debt.

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
    error('PerturbationModesV77:InvalidInput', ...
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

labelRecords = repmat(emptyLabelRecord(), 1, 0);
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
    existenceCommon = mean(existenceDelta) ^ 2;
    existenceCentered = mean(( ...
        existenceDelta - mean(existenceDelta)) .^ 2);
    spatialCommon = 0;
    spatialCentered = 0;
    if covarianceWeight > eps
        covariance = covarianceNumerator / covarianceWeight;
        covariance = (covariance + covariance') / 2 + ...
            regularization * eye(2);
        commonDelta = mean(positionDelta, 2);
        centeredDelta = positionDelta - commonDelta;
        spatialCommon = quadraticEnergy(commonDelta, covariance);
        nodeSpatial = zeros(1, nodeCount);
        for nodeIdx = 1:nodeCount
            nodeSpatial(nodeIdx) = quadraticEnergy( ...
                centeredDelta(:, nodeIdx), covariance);
        end
        spatialCentered = mean(nodeSpatial);
    end
    record = emptyLabelRecord();
    record.label = label;
    record.salience = salience;
    record.existenceCommonEnergy = existenceCommon;
    record.existenceCenteredEnergy = existenceCentered;
    record.spatialCommonEnergy = spatialCommon;
    record.spatialCenteredEnergy = spatialCentered;
    record.commonEnergy = existenceCommon + spatialCommon;
    record.centeredEnergy = existenceCentered + spatialCentered;
    record.totalEnergy = record.commonEnergy + record.centeredEnergy;
    labelRecords(end + 1) = record; %#ok<AGROW>
end

weights = [labelRecords.salience];
if isempty(weights) || sum(weights) <= eps
    aggregate = emptyAggregate();
else
    weights = weights / sum(weights);
    aggregate = emptyAggregate();
    aggregate.labelCount = numel(labelRecords);
    aggregate.existenceCommonEnergy = sum( ...
        weights .* [labelRecords.existenceCommonEnergy]);
    aggregate.existenceCenteredEnergy = sum( ...
        weights .* [labelRecords.existenceCenteredEnergy]);
    aggregate.spatialCommonEnergy = sum( ...
        weights .* [labelRecords.spatialCommonEnergy]);
    aggregate.spatialCenteredEnergy = sum( ...
        weights .* [labelRecords.spatialCenteredEnergy]);
    aggregate.commonEnergy = sum( ...
        weights .* [labelRecords.commonEnergy]);
    aggregate.centeredEnergy = sum( ...
        weights .* [labelRecords.centeredEnergy]);
    aggregate.totalEnergy = aggregate.commonEnergy + ...
        aggregate.centeredEnergy;
    aggregate.commonEnergyFraction = aggregate.commonEnergy / ...
        max(aggregate.totalEnergy, eps);
    aggregate.centeredEnergyFraction = aggregate.centeredEnergy / ...
        max(aggregate.totalEnergy, eps);
end

result = struct();
result.contractVersion = 'lmb-arm-perturbation-modes-v77-v1';
result.nodeCount = nodeCount;
result.labelRecords = labelRecords;
result.aggregate = aggregate;
result.covarianceRegularization = regularization;
result.commonModeUnpenalized = true;
result.truthUsed = false;
result.futureMeasurementUsed = false;
end

function energy = quadraticEnergy(vector, covariance)
energy = vector' * (covariance \ vector);
if ~isfinite(energy) || energy < -1e-10
    error('PerturbationModesV77:InvalidEnergy', ...
        'A perturbation mode energy is invalid.');
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

function record = emptyLabelRecord()
record = struct( ...
    'label', zeros(2, 1), ...
    'salience', NaN, ...
    'existenceCommonEnergy', NaN, ...
    'existenceCenteredEnergy', NaN, ...
    'spatialCommonEnergy', NaN, ...
    'spatialCenteredEnergy', NaN, ...
    'commonEnergy', NaN, ...
    'centeredEnergy', NaN, ...
    'totalEnergy', NaN);
end

function aggregate = emptyAggregate()
aggregate = struct( ...
    'labelCount', 0, ...
    'existenceCommonEnergy', 0, ...
    'existenceCenteredEnergy', 0, ...
    'spatialCommonEnergy', 0, ...
    'spatialCenteredEnergy', 0, ...
    'commonEnergy', 0, ...
    'centeredEnergy', 0, ...
    'totalEnergy', 0, ...
    'commonEnergyFraction', 0, ...
    'centeredEnergyFraction', 0);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
