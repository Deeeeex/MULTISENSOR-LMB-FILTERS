function metrics = computeFormationLabelRepairPressure( ...
        fusedPosteriorBySensor, localPosteriorBySensor, ...
        sensorGroupIds, model, currentTime, options)
% COMPUTEFORMATIONLABELREPAIRPRESSURE Online local-to-fused support debt.
%
% A sensor needs no truth or alternative trajectory to ask whether its
% current fusion step displaced label information supported by its own
% measurement update.  For every locally supported Bernoulli label, this
% function reuses the installed handover-rescue feature with the fused
% object as receiver and the local object as source.  The resulting label
% scores are association-weighted within a sensor and mean-tail aggregated
% within a formation.  Numeric label values are used only for exact object
% matching and never enter the score.
%
% The formation aggregate is a trigger feature, not a tracking-loss bound.
% A runtime controller must still charge the scalar member-to-coordinator
% messages used to form it and project any requested repair through the
% complete-label safety and communication constraints.

if nargin < 6 || isempty(options)
    options = struct();
end
sensorCount = numel(localPosteriorBySensor);
groupIds = reshape(sensorGroupIds, 1, []);
activeExistenceThreshold = getField( ...
    options, 'activeExistenceThreshold', 1e-2);
minimumAssociationSupport = getField( ...
    options, 'minimumAssociationSupport', 0.05);
sensorMeanWeight = getField(options, 'sensorMeanWeight', 0.5);
formationMeanWeight = getField(options, 'formationMeanWeight', 0.5);
if ~iscell(fusedPosteriorBySensor) || ...
        numel(fusedPosteriorBySensor) ~= sensorCount || ...
        sensorCount < 1 || numel(groupIds) ~= sensorCount || ...
        any(~isfinite(groupIds)) || any(groupIds ~= round(groupIds)) || ...
        any(groupIds < 1) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime < 1 || currentTime ~= round(currentTime) || ...
        ~isscalar(activeExistenceThreshold) || ...
        ~isfinite(activeExistenceThreshold) || ...
        activeExistenceThreshold < 0 || ...
        activeExistenceThreshold >= 1 || ...
        ~isscalar(minimumAssociationSupport) || ...
        ~isfinite(minimumAssociationSupport) || ...
        minimumAssociationSupport < 0 || ...
        minimumAssociationSupport > 1 || ...
        ~validUnitWeight(sensorMeanWeight) || ...
        ~validUnitWeight(formationMeanWeight)
    error('FormationLabelRepairPressure:InvalidInput', ...
        'The online repair-pressure inputs or options are malformed.');
end

sensorMeanScore = zeros(1, sensorCount);
sensorTailScore = zeros(1, sensorCount);
sensorPressure = zeros(1, sensorCount);
sensorLocalSupportMass = zeros(1, sensorCount);
sensorSupportedLabelCount = zeros(1, sensorCount);
for sensorIdx = 1:sensorCount
    localObjects = reshape(localPosteriorBySensor{sensorIdx}, 1, []);
    fusedObjects = reshape(fusedPosteriorBySensor{sensorIdx}, 1, []);
    labelScores = zeros(1, 0);
    labelWeights = zeros(1, 0);
    for objectIdx = 1:numel(localObjects)
        localObject = localObjects(objectIdx);
        if localObject.numberOfGmComponents < 1 || ...
                localObject.r < activeExistenceThreshold
            continue;
        end
        associationSupport = max([ ...
            boundedScalar(localObject, 'associationConfidence'), ...
            boundedScalar(localObject, 'detectionAssociationMass')]);
        if associationSupport < minimumAssociationSupport
            continue;
        end
        fusedObject = findLabelObject(fusedObjects, localObject);
        features = computeObservableLmbLabelTransferFeatures( ...
            fusedObject, localObject, model, sensorIdx, sensorIdx, ...
            currentTime);
        weight = min(max(localObject.r, 0), 1) * associationSupport;
        labelScores(end + 1) = ...
            features.handoverRescueScore; %#ok<AGROW>
        labelWeights(end + 1) = weight; %#ok<AGROW>
    end
    supportMass = sum(labelWeights);
    sensorLocalSupportMass(sensorIdx) = supportMass;
    sensorSupportedLabelCount(sensorIdx) = numel(labelScores);
    if supportMass <= eps
        continue;
    end
    sensorMeanScore(sensorIdx) = ...
        sum(labelWeights .* labelScores) / supportMass;
    sensorTailScore(sensorIdx) = max(labelScores);
    sensorPressure(sensorIdx) = sensorMeanWeight * ...
        sensorMeanScore(sensorIdx) + (1 - sensorMeanWeight) * ...
        sensorTailScore(sensorIdx);
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
metrics.contractVersion = 'formation-label-repair-pressure-v188-v1';
metrics.currentTime = currentTime;
metrics.activeExistenceThreshold = activeExistenceThreshold;
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

function object = findLabelObject(objects, target)
object = [];
for objectIdx = 1:numel(objects)
    candidate = objects(objectIdx);
    if candidate.numberOfGmComponents > 0 && ...
            candidate.birthTime == target.birthTime && ...
            candidate.birthLocation == target.birthLocation
        object = candidate;
        return;
    end
end
end

function value = boundedScalar(data, name)
value = 0;
if isstruct(data) && isfield(data, name) && ...
        isscalar(data.(name)) && isfinite(data.(name))
    value = min(max(data.(name), 0), 1);
end
end

function valid = validUnitWeight(value)
valid = isscalar(value) && isfinite(value) && ...
    value >= 0 && value <= 1;
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
