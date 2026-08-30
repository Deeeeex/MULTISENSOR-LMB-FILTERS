function metrics = computeFormationLabelRepairPressureFromSynopsisV188( ...
        cache, sensorGroupIds, options)
% COMPUTEFORMATIONLABELREPAIRPRESSUREFROMSYNOPSISV188 Cached support debt.

if nargin < 3 || isempty(options)
    options = struct();
end
validateCache(cache);
groupIds = reshape(sensorGroupIds, 1, []);
sensorCount = cache.sensorCount;
minimumAssociationSupport = getField( ...
    options, 'minimumAssociationSupport', 0.05);
sensorMeanWeight = getField(options, 'sensorMeanWeight', 0.5);
formationMeanWeight = getField(options, 'formationMeanWeight', 0.5);
if numel(groupIds) ~= sensorCount || ...
        any(~isfinite(groupIds)) || any(groupIds ~= round(groupIds)) || ...
        any(groupIds < 1) || ...
        ~validUnitWeight(minimumAssociationSupport) || ...
        ~validUnitWeight(sensorMeanWeight) || ...
        ~validUnitWeight(formationMeanWeight)
    error('FormationRepairPressureSynopsisV188:InvalidInput', ...
        'The synopsis pressure inputs are malformed.');
end

sensorMeanScore = zeros(1, sensorCount);
sensorTailScore = zeros(1, sensorCount);
sensorPressure = zeros(1, sensorCount);
sensorLocalSupportMass = zeros(1, sensorCount);
sensorSupportedLabelCount = zeros(1, sensorCount);
for sensorIdx = 1:sensorCount
    local = cache.localBySensor{sensorIdx};
    fused = cache.fusedBySensor{sensorIdx};
    supported = find(local.associationSupport >= ...
        minimumAssociationSupport);
    scores = zeros(1, numel(supported));
    weights = zeros(1, numel(supported));
    for position = 1:numel(supported)
        localIdx = supported(position);
        fusedIdx = findLabel(fused.labels, local.labels(:, localIdx));
        scores(position) = transferScore( ...
            fused, fusedIdx, local, localIdx, cache.positionCutoff);
        weights(position) = local.existence(localIdx) * ...
            local.associationSupport(localIdx);
    end
    supportMass = sum(weights);
    sensorLocalSupportMass(sensorIdx) = supportMass;
    sensorSupportedLabelCount(sensorIdx) = numel(scores);
    if supportMass <= eps
        continue;
    end
    sensorMeanScore(sensorIdx) = sum(weights .* scores) / supportMass;
    sensorTailScore(sensorIdx) = max(scores);
    sensorPressure(sensorIdx) = sensorMeanWeight * ...
        sensorMeanScore(sensorIdx) + ...
        (1 - sensorMeanWeight) * sensorTailScore(sensorIdx);
end

formationIds = unique(groupIds, 'stable');
formationCount = numel(formationIds);
formationMeanScore = zeros(1, formationCount);
formationTailScore = zeros(1, formationCount);
formationPressure = zeros(1, formationCount);
formationSupportedSensorFraction = zeros(1, formationCount);
formationSupportedLabelCount = zeros(1, formationCount);
for formationIdx = 1:formationCount
    members = find(groupIds == formationIds(formationIdx));
    memberPressure = sensorPressure(members);
    formationMeanScore(formationIdx) = mean(memberPressure);
    formationTailScore(formationIdx) = max(memberPressure);
    formationPressure(formationIdx) = formationMeanWeight * ...
        formationMeanScore(formationIdx) + ...
        (1 - formationMeanWeight) * formationTailScore(formationIdx);
    formationSupportedSensorFraction(formationIdx) = mean( ...
        sensorLocalSupportMass(members) > eps);
    formationSupportedLabelCount(formationIdx) = sum( ...
        sensorSupportedLabelCount(members));
end

metrics = struct();
metrics.contractVersion = ...
    'formation-label-repair-pressure-synopsis-v188-v1';
metrics.currentTime = cache.currentTime;
metrics.minimumAssociationSupport = minimumAssociationSupport;
metrics.sensorMeanWeight = sensorMeanWeight;
metrics.formationMeanWeight = formationMeanWeight;
metrics.sensorMeanScore = sensorMeanScore;
metrics.sensorTailScore = sensorTailScore;
metrics.sensorPressure = sensorPressure;
metrics.sensorLocalSupportMass = sensorLocalSupportMass;
metrics.sensorSupportedLabelCount = sensorSupportedLabelCount;
metrics.formationIds = formationIds;
metrics.formationMeanScore = formationMeanScore;
metrics.formationTailScore = formationTailScore;
metrics.formationPressure = formationPressure;
metrics.formationSupportedSensorFraction = ...
    formationSupportedSensorFraction;
metrics.formationSupportedLabelCount = formationSupportedLabelCount;
metrics.truthUsed = false;
metrics.futureInformationUsed = false;
metrics.numericLabelIdentifiersUsedAsFeatures = false;
metrics.requiresChargedFormationAggregation = true;
metrics.trackingLossBoundClaimed = false;
end

function score = transferScore( ...
        receiver, receiverIdx, source, sourceIdx, positionCutoff)
receiverPresent = receiverIdx > 0;
receiverExistence = 0;
receiverTrace = positionCutoff^2;
receiverEvidence = 0;
receiverOpportunity = 0;
receiverRisk = 0.5;
disagreement = 0;
if receiverPresent
    receiverExistence = receiver.existence(receiverIdx);
    receiverTrace = receiver.positionTrace(receiverIdx);
    receiverEvidence = receiver.evidenceQuality(receiverIdx);
    receiverOpportunity = receiver.observationOpportunity(receiverIdx);
    receiverRisk = receiver.bayesRisk(receiverIdx);
    delta = receiver.positionMean(:, receiverIdx) - ...
        source.positionMean(:, sourceIdx);
    mahalanobis = 2 * sum(delta .^ 2) / max( ...
        receiverTrace + source.positionTrace(sourceIdx), eps);
    disagreement = 1 - exp(-0.5 * min(mahalanobis, 100));
end
sourceExistence = source.existence(sourceIdx);
sourceTrace = source.positionTrace(sourceIdx);
sourceEvidence = source.evidenceQuality(sourceIdx);
sourceOpportunity = source.observationOpportunity(sourceIdx);
sourceRisk = source.bayesRisk(sourceIdx);
existenceGain = max(sourceExistence - receiverExistence, 0);
precisionGain = max(log((receiverTrace + eps) / ...
    (sourceTrace + eps)), 0);
precisionGain = 1 - exp(-min(precisionGain, 20));
evidenceGap = max(sourceEvidence - receiverEvidence, 0);
opportunityGap = max(sourceOpportunity - receiverOpportunity, 0);
sourcePrecisionQuality = 1 / (1 + ...
    sourceTrace / max(positionCutoff^2, eps));
sourceQuality = sourceExistence * ( ...
    0.40 * sourceEvidence + 0.40 * sourceOpportunity + ...
    0.20 * sourcePrecisionQuality);
riskReduction = receiverRisk - sourceRisk;
score = 0.30 * existenceGain + ...
    0.20 * precisionGain + ...
    0.20 * evidenceGap + ...
    0.15 * opportunityGap + ...
    0.15 * sourceQuality * disagreement + ...
    0.25 * max(riskReduction, 0);
end

function index = findLabel(labels, label)
index = 0;
if isempty(labels)
    return;
end
match = find(all(labels == label, 1), 1);
if ~isempty(match)
    index = match;
end
end

function validateCache(cache)
if ~isstruct(cache) || ~isfield(cache, 'contractVersion') || ...
        ~strcmp(cache.contractVersion, ...
        'formation-repair-light-synopsis-cache-v188-v1')
    error('FormationRepairPressureSynopsisV188:CacheContractDrift', ...
        'A frozen V188 light-synopsis cache is required.');
end
end

function valid = validUnitWeight(value)
valid = isscalar(value) && isfinite(value) && value >= 0 && value <= 1;
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
