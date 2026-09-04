function summary = summarizeFormationLmbRiskModesV259( ...
        localPosteriorBySensor, groupIds, model, options)
% SUMMARIZEFORMATIONLMBRISKMODESV259 Split support and localization risk.

if nargin < 4 || isempty(options)
    options = struct();
end
protocol = getRiskDecomposedHorizonV259Protocol();
activeThreshold = getField(options, 'activeExistenceThreshold', ...
    protocol.activeExistenceThreshold);
networkSupportThreshold = getField(options, ...
    'networkSupportThreshold', protocol.networkSupportThreshold);
localizationTailFraction = getField(options, ...
    'localizationTailFraction', protocol.localizationTailFraction);
minimumLocalizationCoverage = getField(options, ...
    'minimumLocalizationFormationCoverage', ...
    protocol.minimumLocalizationFormationCoverage);
localizationSensorLowerQuantile = getField(options, ...
    'localizationSensorLowerQuantile', ...
    protocol.localizationSensorLowerQuantile);
supportTailFraction = getField(options, ...
    'supportTailFraction', protocol.supportTailFraction);
existenceClip = getField(options, ...
    'existenceClip', protocol.existenceClip);
positionCutoff = resolvePositionCutoff(model, options);
validateInputs(localPosteriorBySensor, groupIds, model, ...
    activeThreshold, networkSupportThreshold, ...
    localizationTailFraction, supportTailFraction, existenceClip, ...
    minimumLocalizationCoverage, localizationSensorLowerQuantile);

nodeCount = numel(localPosteriorBySensor);
groupIds = reshape(groupIds, 1, []);
groups = unique(groupIds, 'stable');
formationCount = numel(groups);
sensorSummaries = cell(1, nodeCount);
labels = zeros(2, 0);
for sensorIdx = 1:nodeCount
    sensorSummaries{sensorIdx} = ...
        summarizeLmbPosteriorForDisagreement( ...
            localPosteriorBySensor{sensorIdx}, model);
    labels = [labels, sensorSummaries{sensorIdx}.labels]; %#ok<AGROW>
end
if isempty(labels)
    labels = zeros(2, 0);
else
    labels = unique(labels', 'rows')';
end
labelCount = size(labels, 2);

existenceBySensorLabel = zeros(nodeCount, labelCount);
normalizedPositionMseBySensorLabel = nan(nodeCount, labelCount);
activeLabelCountBySensor = zeros(1, nodeCount);
for sensorIdx = 1:nodeCount
    sensor = sensorSummaries{sensorIdx};
    activeLabelCountBySensor(sensorIdx) = ...
        nnz(sensor.existence >= activeThreshold);
    for localLabelIdx = 1:size(sensor.labels, 2)
        labelIdx = find(all(labels == ...
            sensor.labels(:, localLabelIdx), 1), 1);
        existenceBySensorLabel(sensorIdx, labelIdx) = ...
            sensor.existence(localLabelIdx);
        covariance = ...
            sensor.positionCovariance(:, :, localLabelIdx);
        normalizedPositionMseBySensorLabel(sensorIdx, labelIdx) = ...
            min(max(trace(covariance), 0) / positionCutoff^2, 1);
    end
end

formationMedianExistence = zeros(formationCount, labelCount);
formationActiveCoverage = zeros(formationCount, labelCount);
labelLocalizationRisk = nan(formationCount, labelCount);
formationMedianActiveLabelCount = zeros(1, formationCount);
for formationIdx = 1:formationCount
    members = find(groupIds == groups(formationIdx));
    formationMedianActiveLabelCount(formationIdx) = ...
        median(activeLabelCountBySensor(members));
    formationMedianExistence(formationIdx, :) = ...
        median(existenceBySensorLabel(members, :), 1);
    active = existenceBySensorLabel(members, :) >= activeThreshold;
    formationActiveCoverage(formationIdx, :) = mean(active, 1);
    for labelIdx = 1:labelCount
        supported = active(:, labelIdx) & isfinite( ...
            normalizedPositionMseBySensorLabel(members, labelIdx));
        if formationActiveCoverage(formationIdx, labelIdx) < ...
                minimumLocalizationCoverage || ...
                ~any(supported)
            continue;
        end
        memberMse = normalizedPositionMseBySensorLabel( ...
            members(supported), labelIdx);
        labelLocalizationRisk(formationIdx, labelIdx) = ...
            quantile(memberMse, localizationSensorLowerQuantile);
    end
end

networkReferenceExistence = max(formationMedianExistence, [], 1);
networkSupportedLabelMask = ...
    networkReferenceExistence >= networkSupportThreshold;
labelSupportDeficit = zeros(formationCount, labelCount);
for formationIdx = 1:formationCount
    for labelIdx = find(networkSupportedLabelMask)
        reference = networkReferenceExistence(labelIdx);
        candidate = formationMedianExistence(formationIdx, labelIdx);
        if candidate >= reference
            continue;
        end
        divergence = bernoulliKld(reference, candidate, existenceClip);
        labelSupportDeficit(formationIdx, labelIdx) = ...
            1 - exp(-divergence);
    end
end

localizationMeanRisk = zeros(1, formationCount);
localizationTailRisk = zeros(1, formationCount);
supportMeanRisk = zeros(1, formationCount);
supportTailRisk = zeros(1, formationCount);
networkSupportCoverage = ones(1, formationCount);
highestLocalizationRiskLabelIndex = zeros(1, formationCount);
highestSupportRiskLabelIndex = zeros(1, formationCount);
for formationIdx = 1:formationCount
    localization = labelLocalizationRisk(formationIdx, :);
    localizationIndices = find(isfinite(localization));
    if ~isempty(localizationIndices)
        localizationValues = localization(localizationIndices);
        localizationMeanRisk(formationIdx) = mean(localizationValues);
        localizationTailRisk(formationIdx) = upperTailMean( ...
            localizationValues, localizationTailFraction);
        [~, localMaximumIdx] = max(localizationValues);
        highestLocalizationRiskLabelIndex(formationIdx) = ...
            localizationIndices(localMaximumIdx);
    end
    supportIndices = find(networkSupportedLabelMask);
    if ~isempty(supportIndices)
        supportValues = ...
            labelSupportDeficit(formationIdx, supportIndices);
        supportMeanRisk(formationIdx) = mean(supportValues);
        supportTailRisk(formationIdx) = upperTailMean( ...
            supportValues, supportTailFraction);
        networkSupportCoverage(formationIdx) = mean( ...
            formationMedianExistence(formationIdx, supportIndices) >= ...
                activeThreshold);
        [~, localMaximumIdx] = max(supportValues);
        highestSupportRiskLabelIndex(formationIdx) = ...
            supportIndices(localMaximumIdx);
    end
end
medianSupportMeanRisk = median(supportMeanRisk);
supportProminenceRisk = max( ...
    supportMeanRisk - medianSupportMeanRisk, 0);
medianNetworkSupportCoverage = median(networkSupportCoverage);
relativeNetworkSupportCoverageDeficit = max( ...
    medianNetworkSupportCoverage - networkSupportCoverage, 0) ./ ...
    max(medianNetworkSupportCoverage, eps);
medianActiveLabelCount = median(formationMedianActiveLabelCount);
relativeActiveLabelCountDeficit = max( ...
    medianActiveLabelCount - formationMedianActiveLabelCount, 0) ./ ...
    max(medianActiveLabelCount, 1);

summary = struct();
summary.contractVersion = ...
    'formation-lmb-risk-modes-v259-summary-v1';
summary.formationIds = groups;
summary.labels = labels;
summary.existenceBySensorLabel = existenceBySensorLabel;
summary.normalizedPositionMseBySensorLabel = ...
    normalizedPositionMseBySensorLabel;
summary.activeLabelCountBySensor = activeLabelCountBySensor;
summary.formationMedianActiveLabelCount = ...
    formationMedianActiveLabelCount;
summary.formationMedianExistence = formationMedianExistence;
summary.formationActiveCoverage = formationActiveCoverage;
summary.networkReferenceExistence = networkReferenceExistence;
summary.networkSupportedLabelMask = networkSupportedLabelMask;
summary.labelLocalizationRiskByFormation = labelLocalizationRisk;
summary.labelSupportDeficitByFormation = labelSupportDeficit;
summary.localizationMeanRiskByFormation = localizationMeanRisk;
summary.localizationTailRiskByFormation = localizationTailRisk;
summary.supportMeanRiskByFormation = supportMeanRisk;
summary.supportTailRiskByFormation = supportTailRisk;
summary.supportProminenceRiskByFormation = supportProminenceRisk;
summary.networkSupportCoverageByFormation = networkSupportCoverage;
summary.relativeNetworkSupportCoverageDeficitByFormation = ...
    relativeNetworkSupportCoverageDeficit;
summary.relativeActiveLabelCountDeficitByFormation = ...
    relativeActiveLabelCountDeficit;
summary.highestLocalizationRiskLabelIndexByFormation = ...
    highestLocalizationRiskLabelIndex;
summary.highestSupportRiskLabelIndexByFormation = ...
    highestSupportRiskLabelIndex;
summary.positionCutoff = positionCutoff;
summary.activeExistenceThreshold = activeThreshold;
summary.networkSupportThreshold = networkSupportThreshold;
summary.localizationTailFraction = localizationTailFraction;
summary.minimumLocalizationFormationCoverage = ...
    minimumLocalizationCoverage;
summary.localizationSensorLowerQuantile = ...
    localizationSensorLowerQuantile;
summary.supportTailFraction = supportTailFraction;
summary.existenceClip = existenceClip;
summary.truthUsed = false;
summary.futureMeasurementUsed = false;
summary.currentNetworkPosteriorSynopsisUsed = true;
summary.distributedControlSynopsisCostIncluded = false;
summary.evidenceBoundary = [ ...
    'This passive summary is causal with respect to the captured current ', ...
    'posteriors, but its network-reference support term assumes access to ', ...
    'all current formation existence summaries. A deployed distributed ', ...
    'controller must define and charge that control synopsis.'];
end

function value = bernoulliKld(reference, candidate, clip)
reference = min(max(reference, clip), 1 - clip);
candidate = min(max(candidate, clip), 1 - clip);
value = reference * log(reference / candidate) + ...
    (1 - reference) * log((1 - reference) / (1 - candidate));
value = max(value, 0);
end

function value = upperTailMean(values, fraction)
values = sort(reshape(values, 1, []), 'descend');
if isempty(values)
    value = 0;
    return;
end
count = max(1, ceil(fraction * numel(values)));
value = mean(values(1:count));
end

function value = resolvePositionCutoff(model, options)
value = getField(options, 'positionCutoff', NaN);
if ~isfinite(value) && isfield(model, 'ospaParameters') && ...
        isstruct(model.ospaParameters) && ...
        isfield(model.ospaParameters, 'eC')
    value = model.ospaParameters.eC;
end
if ~isscalar(value) || ~isfinite(value) || value <= 0
    error('RiskDecomposedV259:MissingPositionCutoff', ...
        'The model must provide a positive position OSPA cutoff.');
end
end

function validateInputs(posteriors, groupIds, model, activeThreshold, ...
        supportThreshold, localizationTail, supportTail, clip, ...
        minimumLocalizationCoverage, sensorLowerQuantile)
nodeCount = numel(posteriors);
if ~iscell(posteriors) || nodeCount < 1 || ...
        ~isstruct(model) || ~isscalar(model) || ...
        ~isfield(model, 'xDimension') || model.xDimension < 2 || ...
        numel(groupIds) ~= nodeCount || any(~isfinite(groupIds)) || ...
        any([activeThreshold, supportThreshold] <= 0) || ...
        any([activeThreshold, supportThreshold] >= 1) || ...
        any([localizationTail, supportTail] <= 0) || ...
        any([localizationTail, supportTail] > 1) || ...
        ~isscalar(clip) || ~isfinite(clip) || clip <= 0 || clip >= 0.5
    error('RiskDecomposedV259:InvalidRiskInput', ...
        'The posterior, formation or risk options are malformed.');
end
if ~isscalar(minimumLocalizationCoverage) || ...
        ~isfinite(minimumLocalizationCoverage) || ...
        minimumLocalizationCoverage <= 0.5 || ...
        minimumLocalizationCoverage > 1 || ...
        ~isscalar(sensorLowerQuantile) || ...
        ~isfinite(sensorLowerQuantile) || ...
        sensorLowerQuantile < 0 || sensorLowerQuantile > 0.5
    error('RiskDecomposedV259:InvalidLocalizationAggregation', ...
        'The V259 localization coverage or sensor quantile is invalid.');
end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
