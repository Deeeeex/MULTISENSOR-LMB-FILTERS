function result = computeReplacementInnovationEnergyV75( ...
        localPosteriors, slotTriples, groupIds, model, ...
        commConfig, currentTime, options)
% COMPUTEREPLACEMENTINNOVATIONENERGYV75 Sender-replacement spatial risk.
%
% slotTriples rows are [receiver, incumbent sender, candidate sender].
% Each label energy is the normalized squared distance between incumbent
% and candidate GM moments.  The receiver and both senders must support the
% label through positive existence probability.  Low delivery probability
% cannot hide a harmful conditional replacement.

if nargin < 7 || isempty(options)
    options = struct();
end
nodeCount = numel(localPosteriors);
groupIds = reshape(groupIds, 1, []);
energyLimit = getField(options, ...
    'conditionalEnergyLimit', 5.991464547107979);
regularization = getField(options, ...
    'covarianceRegularization', 1e-9);
if size(slotTriples, 2) ~= 3 || isempty(slotTriples) || ...
        any(~isfinite(slotTriples(:))) || ...
        any(mod(slotTriples(:), 1) ~= 0) || ...
        any(slotTriples(:) < 1) || any(slotTriples(:) > nodeCount) || ...
        numel(groupIds) ~= nodeCount || ...
        ~isscalar(energyLimit) || ~isfinite(energyLimit) || ...
        energyLimit <= 0 || ~isscalar(regularization) || ...
        ~isfinite(regularization) || regularization < 0
    error('ReplacementInnovationV75:InvalidInput', ...
        'Replacement innovation inputs or options are invalid.');
end

summaries = cell(1, nodeCount);
for sensorIdx = 1:nodeCount
    summaries{sensorIdx} = ...
        summarizeLmbPosteriorForDisagreement( ...
            localPosteriors{sensorIdx}, model);
end
slotRecords = repmat(emptySlotRecord(), 1, size(slotTriples, 1));
for slotIdx = 1:size(slotTriples, 1)
    receiverIdx = slotTriples(slotIdx, 1);
    incumbentIdx = slotTriples(slotIdx, 2);
    candidateIdx = slotTriples(slotIdx, 3);
    slotRecords(slotIdx) = scoreSlot( ...
        receiverIdx, incumbentIdx, candidateIdx, summaries, ...
        groupIds, commConfig, currentTime, energyLimit, regularization);
end

formations = unique([slotRecords.formationId], 'stable');
formationRecords = repmat(emptyFormationRecord(), 1, numel(formations));
for formationIdx = 1:numel(formations)
    formationId = formations(formationIdx);
    selected = [slotRecords.formationId] == formationId;
    energies = [slotRecords(selected).conditionalEnergy];
    expected = [slotRecords(selected).reliabilityWeightedEnergy];
    coverage = [slotRecords(selected).eligibleLabelCount] > 0;
    record = emptyFormationRecord();
    record.formationId = formationId;
    record.slotCount = nnz(selected);
    record.coveredSlotCount = nnz(coverage);
    if all(coverage)
        record.maximumConditionalEnergy = max(energies);
        record.maximumReliabilityWeightedEnergy = max(expected);
        record.safe = record.maximumConditionalEnergy <= ...
            energyLimit + 1e-12;
    end
    formationRecords(formationIdx) = record;
end

result = struct();
result.contractVersion = 'replacement-innovation-energy-v75-v1';
result.slotTriples = slotTriples;
result.slotRecords = slotRecords;
result.formationIds = formations;
result.formationRecords = formationRecords;
result.conditionalEnergyLimit = energyLimit;
result.covarianceRegularization = regularization;
result.allSlotsCovered = all([slotRecords.eligibleLabelCount] > 0);
result.allFormationsSafe = result.allSlotsCovered && ...
    all([formationRecords.safe]);
result.truthUsed = false;
result.futureMeasurementUsed = false;
result.futureOutcomeUsed = false;
end

function record = scoreSlot( ...
        receiverIdx, incumbentIdx, candidateIdx, summaries, ...
        groupIds, commConfig, currentTime, energyLimit, regularization)
receiver = summaries{receiverIdx};
incumbent = summaries{incumbentIdx};
candidate = summaries{candidateIdx};
labels = commonLabels(receiver.labels, ...
    incumbent.labels, candidate.labels);
energies = zeros(1, 0);
weights = zeros(1, 0);
usedLabels = zeros(2, 0);
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    receiverLabelIdx = findLabel(receiver.labels, label);
    incumbentLabelIdx = findLabel(incumbent.labels, label);
    candidateLabelIdx = findLabel(candidate.labels, label);
    receiverExistence = receiver.existence(receiverLabelIdx);
    incumbentExistence = incumbent.existence(incumbentLabelIdx);
    candidateExistence = candidate.existence(candidateLabelIdx);
    weight = receiverExistence * min( ...
        incumbentExistence, candidateExistence);
    if ~isfinite(weight) || weight <= eps
        continue;
    end
    delta = candidate.positionMean(:, candidateLabelIdx) - ...
        incumbent.positionMean(:, incumbentLabelIdx);
    covariance = ...
        incumbent.positionCovariance(:, :, incumbentLabelIdx) + ...
        candidate.positionCovariance(:, :, candidateLabelIdx);
    covariance = (covariance + covariance') / 2 + ...
        regularization * eye(2);
    energy = delta' * (covariance \ delta);
    if ~isfinite(energy) || energy < -1e-10
        error('ReplacementInnovationV75:InvalidEnergy', ...
            'A slot label produced invalid innovation energy.');
    end
    usedLabels(:, end + 1) = label; %#ok<AGROW>
    weights(end + 1) = weight; %#ok<AGROW>
    energies(end + 1) = max(energy, 0); %#ok<AGROW>
end

record = emptySlotRecord();
record.receiverIdx = receiverIdx;
record.incumbentSenderIdx = incumbentIdx;
record.candidateSenderIdx = candidateIdx;
record.formationId = groupIds(receiverIdx);
record.eligibleLabels = usedLabels;
record.eligibleLabelCount = numel(weights);
record.labelWeights = weights;
record.labelEnergies = energies;
record.linkReliability = linkReliability( ...
    commConfig, candidateIdx, receiverIdx, currentTime);
if ~isempty(weights)
    record.conditionalEnergy = ...
        sum(weights .* energies) / sum(weights);
    record.reliabilityWeightedEnergy = ...
        record.linkReliability * record.conditionalEnergy;
    record.safe = record.conditionalEnergy <= energyLimit + 1e-12;
end
end

function labels = commonLabels(first, second, third)
labels = zeros(2, 0);
for labelIdx = 1:size(first, 2)
    label = first(:, labelIdx);
    if ~isempty(findLabel(second, label)) && ...
            ~isempty(findLabel(third, label))
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

function reliability = linkReliability(config, senderIdx, receiverIdx, time)
if isfield(config, 'pDropByEdge') && ~isempty(config.pDropByEdge)
    if ndims(config.pDropByEdge) >= 3
        timeIdx = min(time, size(config.pDropByEdge, 3));
        drop = config.pDropByEdge(senderIdx, receiverIdx, timeIdx);
    else
        drop = config.pDropByEdge(senderIdx, receiverIdx);
    end
elseif isfield(config, 'pDropBySensor') && ...
        numel(config.pDropBySensor) >= senderIdx
    drop = config.pDropBySensor(senderIdx);
else
    drop = 0;
end
reliability = 1 - min(max(drop, 0), 1);
end

function record = emptySlotRecord()
record = struct( ...
    'receiverIdx', NaN, ...
    'incumbentSenderIdx', NaN, ...
    'candidateSenderIdx', NaN, ...
    'formationId', NaN, ...
    'eligibleLabels', zeros(2, 0), ...
    'eligibleLabelCount', 0, ...
    'labelWeights', zeros(1, 0), ...
    'labelEnergies', zeros(1, 0), ...
    'linkReliability', NaN, ...
    'conditionalEnergy', NaN, ...
    'reliabilityWeightedEnergy', NaN, ...
    'safe', false);
end

function record = emptyFormationRecord()
record = struct( ...
    'formationId', NaN, ...
    'slotCount', 0, ...
    'coveredSlotCount', 0, ...
    'maximumConditionalEnergy', NaN, ...
    'maximumReliabilityWeightedEnergy', NaN, ...
    'safe', false);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
