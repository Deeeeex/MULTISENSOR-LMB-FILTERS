function [allowed, details] = evaluateLabelKlaEtaRetentionV223( ...
        referenceObjects, candidateObjects, candidateDiagnostics, ...
        label, options)
% EVALUATELABELKLAETARETENTIONV223 Receiver-specific route projection.
%
% The candidate fusion must be evaluated with label-KLA diagnostics enabled.
% This function compares its selected-label existence with the ordinary KLA
% result and applies the deterministic eta/log-odds projection.

if nargin < 5 || isempty(options)
    options = struct();
end
label = reshape(label, [], 1);
identityTolerance = getField(options, 'identityTolerance', 1e-8);
if numel(label) ~= 2 || any(~isfinite(label)) || ...
        any(label ~= round(label)) || any(label < 1) || ...
        ~isstruct(candidateDiagnostics) || ...
        ~isfield(candidateDiagnostics, 'labelKlaRecords') || ...
        ~isscalar(identityTolerance) || ~isfinite(identityTolerance) || ...
        identityTolerance < 0
    error('LabelKlaEtaRetentionV223:InvalidInput', ...
        'The receiver-specific eta projection request is malformed.');
end

details = emptyDetails(label);
referenceObject = findLabelObject(referenceObjects, label);
candidateObject = findLabelObject(candidateObjects, label);
record = findLabelRecord(candidateDiagnostics.labelKlaRecords, label);
if isempty(referenceObject)
    details.rejectionReason = 'ordinary-fused-label-missing';
    allowed = false;
    return;
end
if isempty(candidateObject) || isempty(record)
    details.referenceExistence = referenceObject.r;
    details.rejectionReason = 'candidate-label-or-diagnostic-missing';
    allowed = false;
    return;
end

participating = reshape(record.existenceParticipating, 1, []) & ...
    reshape(record.activeExistenceWeights, 1, []) > 0;
inputExistence = reshape(record.inputExistence, 1, []);
existenceWeights = reshape(record.activeExistenceWeights, 1, []);
if numel(participating) ~= numel(inputExistence) || ...
        numel(participating) ~= numel(existenceWeights) || ...
        ~any(participating) || ...
        any(~isfinite(inputExistence(participating)))
    details.referenceExistence = referenceObject.r;
    details.candidateExistence = candidateObject.r;
    details.rejectionReason = 'candidate-diagnostic-incomplete';
    allowed = false;
    return;
end

projectionOptions = options;
[allowed, projection] = ...
    projectLabelKlaExistenceRetentionByEtaV223( ...
        inputExistence(participating), ...
        existenceWeights(participating), ...
        record.spatialLogNormalizer, referenceObject.r, ...
        projectionOptions);
candidateExistenceResidual = ...
    candidateObject.r - projection.candidateExistence;
if abs(record.normalizerIdentityResidual) > identityTolerance || ...
        abs(candidateExistenceResidual) > identityTolerance
    error('LabelKlaEtaRetentionV223:IdentityMismatch', ...
        ['The candidate fusion diagnostics do not satisfy the registered ', ...
         'Bernoulli log-odds identity.']);
end

details.contractVersion = ...
    'receiver-label-kla-eta-retention-evaluation-v223-v1';
details.referenceExistence = referenceObject.r;
details.candidateExistence = candidateObject.r;
details.candidateExistenceResidual = candidateExistenceResidual;
details.normalizerIdentityResidual = ...
    record.normalizerIdentityResidual;
details.mixtureAwareSpatialUsed = record.mixtureAwareSpatialUsed;
details.existenceEtaMode = record.existenceEtaMode;
details.fusedComponentCount = record.fusedComponentCount;
details.projection = projection;
details.allowed = allowed;
if allowed
    details.rejectionReason = '';
else
    details.rejectionReason = 'eta-existence-retention-failed';
end
details.truthUsed = false;
details.futureInformationUsed = false;
end

function object = findLabelObject(objects, label)
object = [];
objects = reshape(objects, 1, []);
for objectIdx = 1:numel(objects)
    candidate = objects(objectIdx);
    if candidate.birthTime == label(1) && ...
            candidate.birthLocation == label(2)
        object = candidate;
        return;
    end
end
end

function record = findLabelRecord(records, label)
record = [];
records = reshape(records, 1, []);
for recordIdx = 1:numel(records)
    if isequal(reshape(records(recordIdx).label, [], 1), label)
        if ~isempty(record)
            error('LabelKlaEtaRetentionV223:DuplicateDiagnostic', ...
                'Only one candidate diagnostic is allowed per label.');
        end
        record = records(recordIdx);
    end
end
end

function details = emptyDetails(label)
details = struct( ...
    'contractVersion', ...
        'receiver-label-kla-eta-retention-evaluation-v223-v1', ...
    'label', reshape(label, 2, 1), ...
    'referenceExistence', NaN, ...
    'candidateExistence', NaN, ...
    'candidateExistenceResidual', NaN, ...
    'normalizerIdentityResidual', NaN, ...
    'mixtureAwareSpatialUsed', false, ...
    'existenceEtaMode', '', ...
    'fusedComponentCount', 0, ...
    'projection', struct(), ...
    'allowed', false, ...
    'rejectionReason', '', ...
    'truthUsed', false, ...
    'futureInformationUsed', false);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
