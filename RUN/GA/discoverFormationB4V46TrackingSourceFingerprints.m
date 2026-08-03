function discovery = ...
    discoverFormationB4V46TrackingSourceFingerprints(options)
% DISCOVERFORMATIONB4V46TRACKINGSOURCEFINGERPRINTS Hash-only discovery.
%
% This is not a source permit.  It materializes registered input objects,
% records their hashes, and discards them without executing either filter.

if nargin < 1 || isempty(options)
    options = struct();
end
allowedFields = {'presets', 'seeds'};
if ~isstruct(options) || ~isscalar(options) || ...
        any(~ismember(fieldnames(options), allowedFields))
    error('FormationB4V46TrackingSource:InvalidOptions', ...
        'The source discovery options are malformed.');
end
registry = getFormationB4V46TrackingSourceRegistry();
presets = getField(options, 'presets', {registry.scenes.presetName});
seeds = reshape(getField(options, 'seeds', registry.orderedSeeds), 1, []);
validateSubset(presets, seeds, registry);

records = repmat(emptyRecord(), 1, numel(presets) * numel(seeds));
cursor = 0;
for presetIdx = 1:numel(presets)
    for seedIdx = 1:numel(seeds)
        cursor = cursor + 1;
        source = materializeSourceCase( ...
            presets{presetIdx}, seeds(seedIdx), registry);
        record = emptyRecord();
        record.caseId = source.caseId;
        record.caseOrdinal = source.caseOrdinal;
        record.presetName = source.presetName;
        record.seed = source.seed;
        record.seedRole = source.seedRole;
        record.primaryMatrix = source.primaryMatrix;
        record.stressMatrix = source.stressMatrix;
        record.deliverySeed = source.deliverySeed;
        record.inputFingerprint = source.inputFingerprint;
        record.runtimeFilterInputFingerprint = ...
            source.runtimeFilterInputFingerprint;
        record.runtimeTargetTrajectoryRealizationAbsent = ...
            source.runtimeTargetTrajectoryRealizationAbsent;
        record.registeredBirthPriorRetained = ...
            source.registeredBirthPriorRetained;
        record.sourceGeometryTruthValidationExecuted = ...
            source.sourceGeometryTruthValidationExecuted;
        record.truthHashedForSealing = source.truthHashedForSealing;
        record.filterExecuted = source.filterExecuted;
        record.estimateVsTruthTrackingMetricComputed = ...
            source.estimateVsTruthTrackingMetricComputed;
        record.sourceEnvelopeCanonicalSha256 = ...
            source.sourceEnvelopeCanonicalSha256;
        record.trackingOutcomeScored = source.trackingOutcomeScored;
        record.canonicalSha256 = ...
            computeCanonicalValueSha256(rmfield(record, ...
                'canonicalSha256'));
        records(cursor) = record;
        fprintf('V46 source fingerprint %d/%d: %s %s\n', ...
            cursor, numel(records), record.caseId, ...
            record.inputFingerprint.canonicalSha256);
        clear source;
    end
end

payload = struct();
payload.contractVersion = ...
    'formation-b4-v46-tracking-source-fingerprint-discovery-v2';
payload.registryId = registry.id;
payload.registryCanonicalSha256 = registry.canonicalSha256;
payload.presets = presets;
payload.seeds = seeds;
payload.records = records;
payload.caseCount = numel(records);
[payload.runtimeEngine, payload.runtimeVersion] = runtimeIdentity();
payload.fullRegistryRequested = ...
    isequal(presets, {registry.scenes.presetName}) && ...
    isequal(seeds, registry.orderedSeeds);
payload.sourceInputFingerprintsFrozen = false;
payload.sourceArtifactPublicationAuthorized = false;
payload.rawSourceObjectsReturned = false;
payload.rawTruthReturned = false;
payload.rawMeasurementsReturned = false;
payload.runtimeFilterModelReturned = false;
payload.runtimeTargetTrajectoryRealizationAbsent = true;
payload.registeredBirthPriorRetained = true;
payload.filterExecuted = false;
payload.stateEstimateGenerated = false;
payload.sourceGeometryTruthValidationExecuted = true;
payload.truthHashedForSealing = true;
payload.estimateVsTruthTrackingMetricComputed = false;
payload.armComparedOrSelected = false;
payload.trackingOutcomeScored = false;
payload.developmentAdvanceDecisionMade = false;
payload.confirmationOpened = false;
payload.validationClaimAllowed = false;
payload.developmentEvidenceOnly = true;
discovery = payload;
discovery.canonicalSha256 = ...
    computeCanonicalValueSha256(payload);
end

function source = materializeSourceCase(presetName, seed, registry)
matching = strcmp({registry.cases.presetName}, presetName) & ...
    [registry.cases.seed] == seed;
if nnz(matching) ~= 1
    error('FormationB4V46TrackingSource:UnknownCase', ...
        'The requested V46 tracking source case is not registered.');
end
caseContract = registry.cases(find(matching, 1));
initialRngState = rng();
restoreRng = onCleanup(@() rng(initialRngState));
inputs = generateDynamicTopologyScenarioInputs(presetName, seed);
if ~inputs.validation.isValid || inputs.seed ~= seed || ...
        ~strcmp(computeCanonicalValueSha256(inputs.config), ...
            caseContract.configSnapshotSha256) || ...
        inputs.config.numberOfSensors ~= caseContract.numberOfSensors || ...
        inputs.config.formationCount ~= caseContract.formationCount || ...
        inputs.config.simulationLength ~= caseContract.simulationLength
    error('FormationB4V46TrackingSource:GeneratedInputDrift', ...
        'The generated source case no longer matches the registry.');
end

identity = buildDynamicTopologyPhysicalIdentityRegistry(inputs.config);
[linkUniforms, deliveryMetadata] = ...
    materializePhysicalUidDirectedDeliveryUniforms( ...
        caseContract.deliverySeed, identity.sensorPhysicalUids, ...
        caseContract.simulationLength);
inputs.commConfig.linkUniforms = linkUniforms;
fingerprint = computeFormationB4V46TrackingInputFingerprint( ...
    inputs, identity, deliveryMetadata, caseContract, registry);
runtimeModel = sanitizeFormationB4V46RuntimeFilterModel( ...
    inputs.model, caseContract, registry);
runtimeFingerprint = ...
    computeFormationB4V46RuntimeFilterInputFingerprint( ...
        runtimeModel, inputs.measurements, ...
        inputs.sensorTrajectories, inputs.neighborMap, ...
        inputs.commConfig, fingerprint, caseContract, registry);

sourcePayload = struct();
sourcePayload.contractVersion = ...
    'formation-b4-v46-materialized-tracking-source-envelope-v3';
sourcePayload.registryId = registry.id;
sourcePayload.registryCanonicalSha256 = registry.canonicalSha256;
sourcePayload.caseId = caseContract.id;
sourcePayload.caseOrdinal = caseContract.ordinal;
sourcePayload.caseCanonicalSha256 = caseContract.caseCanonicalSha256;
sourcePayload.presetName = caseContract.presetName;
sourcePayload.seed = caseContract.seed;
sourcePayload.seedRole = caseContract.seedRole;
sourcePayload.deliverySeed = caseContract.deliverySeed;
sourcePayload.primaryMatrix = caseContract.primaryMatrix;
sourcePayload.stressMatrix = caseContract.stressMatrix;
sourcePayload.focusWindow = caseContract.focusWindow;
sourcePayload.inputFingerprint = fingerprint;
sourcePayload.runtimeFilterInputFingerprint = runtimeFingerprint;
sourcePayload.runtimeTargetTrajectoryRealizationAbsent = true;
sourcePayload.registeredBirthPriorRetained = true;
sourcePayload.rawSourceObjectsReturned = false;
sourcePayload.rawTruthReturned = false;
sourcePayload.rawMeasurementsReturned = false;
sourcePayload.filterExecuted = false;
sourcePayload.stateEstimateGenerated = false;
sourcePayload.sourceGeometryTruthValidationExecuted = true;
sourcePayload.truthHashedForSealing = true;
sourcePayload.estimateVsTruthTrackingMetricComputed = false;
sourcePayload.armComparedOrSelected = false;
sourcePayload.trackingOutcomeScored = false;
sourcePayload.developmentAdvanceDecisionMade = false;
sourcePayload.confirmationOpened = false;
sourcePayload.validationClaimMade = false;
source = sourcePayload;
source.sourceEnvelopeCanonicalSha256 = ...
    computeCanonicalValueSha256(sourcePayload);
clear runtimeModel;
clear restoreRng;
end

function validateSubset(presets, seeds, registry)
registeredPresets = {registry.scenes.presetName};
if ~iscell(presets) || isempty(presets) || ...
        any(~cellfun(@ischar, presets)) || ...
        numel(unique(presets)) ~= numel(presets) || ...
        any(~ismember(presets, registeredPresets)) || ...
        ~isequal(presets, registeredPresets( ...
            ismember(registeredPresets, presets))) || ...
        ~isnumeric(seeds) || ~isreal(seeds) || isempty(seeds) || ...
        any(~isfinite(seeds)) || any(seeds ~= round(seeds)) || ...
        numel(unique(seeds)) ~= numel(seeds) || ...
        any(~ismember(seeds, registry.orderedSeeds)) || ...
        ~isequal(seeds, registry.orderedSeeds( ...
            ismember(registry.orderedSeeds, seeds)))
    error('FormationB4V46TrackingSource:InvalidOptions', ...
        'Only ordered subsets of registered scenes and seeds are allowed.');
end
end

function value = emptyRecord()
value = struct('caseId', '', 'caseOrdinal', NaN, ...
    'presetName', '', 'seed', NaN, 'seedRole', '', ...
    'primaryMatrix', false, 'stressMatrix', false, ...
    'deliverySeed', NaN, 'inputFingerprint', struct(), ...
    'runtimeFilterInputFingerprint', struct(), ...
    'runtimeTargetTrajectoryRealizationAbsent', false, ...
    'registeredBirthPriorRetained', false, ...
    'sourceGeometryTruthValidationExecuted', false, ...
    'truthHashedForSealing', false, 'filterExecuted', false, ...
    'estimateVsTruthTrackingMetricComputed', false, ...
    'trackingOutcomeScored', false, ...
    'sourceEnvelopeCanonicalSha256', '', 'canonicalSha256', '');
end

function [engine, engineVersion] = runtimeIdentity()
if exist('OCTAVE_VERSION', 'builtin') ~= 0
    engine = 'GNU Octave';
    engineVersion = OCTAVE_VERSION;
else
    engine = 'MATLAB';
    engineVersion = version;
end
end

function value = getField(structure, name, defaultValue)
if isfield(structure, name)
    value = structure.(name);
else
    value = defaultValue;
end
end
