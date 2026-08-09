function [updatedObjects, diagnostics, labelEvidence] = ...
        updateLmbWithSensorMeasurementAndEvidence( ...
            predictedObjects, measurements, model, sensorIdx, ...
            currentTime, isScheduledSample, evidenceOptions)
% UPDATELMBWITHSENSORMEASUREMENTANDEVIDENCE V54 local update boundary.
%
% Observation opportunity is computed from the local prediction before the
% measurement update. The returned per-label evidence describes only the
% current local update and does not reuse association diagnostics from a
% previous posterior.

if nargin < 6 || isempty(isScheduledSample)
    isScheduledSample = true;
end
if nargin < 7 || isempty(evidenceOptions)
    evidenceOptions = struct();
end

objectCount = numel(predictedObjects);
opportunities = cell(1, objectCount);
for objectIdx = 1:objectCount
    opportunities{objectIdx} = computeLmbLabelObservationOpportunity( ...
        model, sensorIdx, predictedObjects(objectIdx), currentTime);
end

[updatedObjects, diagnostics] = updateLmbWithSensorMeasurement( ...
    predictedObjects, measurements, model, sensorIdx, currentTime, ...
    isScheduledSample);

% The legacy empty-measurement path updates existence and mixture weights
% but otherwise inherits object fields. Reset association fields here so a
% previous detection cannot be mistaken for current positive support.
if isempty(measurements)
    updatedObjects = resetCurrentAssociationDiagnostics(updatedObjects);
end

template = struct( ...
    'birthTime', 0, ...
    'birthLocation', 0, ...
    'predictedObjectIndex', 0, ...
    'updatedObjectIndex', 0, ...
    'opportunity', struct(), ...
    'evidence', struct(), ...
    'type', '', ...
    'isAdmissibleToSafeReference', false, ...
    'usesTargetTruth', false, ...
    'usesFutureMeasurements', false);
labelEvidence = repmat(template, 1, objectCount);
for objectIdx = 1:objectCount
    predictedObject = predictedObjects(objectIdx);
    updatedObjectIdx = findObjectIndexByLabel( ...
        updatedObjects, predictedObject);
    if updatedObjectIdx > 0
        updatedObject = updatedObjects(updatedObjectIdx);
    else
        updatedObject = [];
    end
    evidence = classifyLmbLabelLocalEvidence( ...
        predictedObject, updatedObject, opportunities{objectIdx}, ...
        isScheduledSample, evidenceOptions);
    labelEvidence(objectIdx).birthTime = predictedObject.birthTime;
    labelEvidence(objectIdx).birthLocation = ...
        predictedObject.birthLocation;
    labelEvidence(objectIdx).predictedObjectIndex = objectIdx;
    labelEvidence(objectIdx).updatedObjectIndex = updatedObjectIdx;
    labelEvidence(objectIdx).opportunity = opportunities{objectIdx};
    labelEvidence(objectIdx).evidence = evidence;
    labelEvidence(objectIdx).type = evidence.type;
    labelEvidence(objectIdx).isAdmissibleToSafeReference = ...
        evidence.isAdmissibleToSafeReference;
end

diagnostics.labelEvidenceContractVersion = ...
    'lmb-local-label-evidence-metadata-v1';
diagnostics.labelEvidenceCount = numel(labelEvidence);
diagnostics.labelEvidenceUsesTargetTruth = false;
diagnostics.labelEvidenceUsesFutureMeasurements = false;
end

function objects = resetCurrentAssociationDiagnostics(objects)
for objectIdx = 1:numel(objects)
    objects(objectIdx).associationEntropy = 0;
    objects(objectIdx).detectionAssociationEntropy = 0;
    objects(objectIdx).detectionAssociationMass = 0;
    objects(objectIdx).associationAmbiguity = 0;
    objects(objectIdx).associationConfidence = 1;
end
end

function index = findObjectIndexByLabel(objects, referenceObject)
index = 0;
for objectIdx = 1:numel(objects)
    if objects(objectIdx).birthTime == referenceObject.birthTime && ...
            objects(objectIdx).birthLocation == ...
                referenceObject.birthLocation
        index = objectIdx;
        return;
    end
end
end
