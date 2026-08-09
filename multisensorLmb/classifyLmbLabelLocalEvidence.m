function evidence = classifyLmbLabelLocalEvidence( ...
        predictedObject, updatedObject, opportunity, ...
        isScheduledSample, options)
% CLASSIFYLMBLABELLOCALEVIDENCE Separate support from missing FoV support.
%
% The rule uses only runtime-observable quantities from one local sensor
% update. A sender-label input is admissible to the receiver-safe reference
% only when it carries positive association support or credible negative
% evidence after a genuine observation opportunity.

if nargin < 5 || isempty(options)
    options = struct();
end
if ~isstruct(predictedObject) || ~isscalar(predictedObject) || ...
        ~isfield(predictedObject, 'r') || ...
        ~isfinite(predictedObject.r) || ...
        predictedObject.r < 0 || predictedObject.r > 1
    error('LmbLocalEvidence:InvalidPrediction', ...
        'A predicted label with a valid existence probability is required.');
end
if isempty(updatedObject)
    updatedExistence = 0;
    detectionAssociationMass = 0;
else
    if ~isstruct(updatedObject) || ~isscalar(updatedObject) || ...
            ~isfield(updatedObject, 'r') || ...
            ~isfinite(updatedObject.r) || ...
            updatedObject.r < 0 || updatedObject.r > 1
        error('LmbLocalEvidence:InvalidUpdate', ...
            'The updated label existence probability must be valid.');
    end
    updatedExistence = updatedObject.r;
    detectionAssociationMass = clamp01(getField( ...
        updatedObject, 'detectionAssociationMass', 0));
end
if ~isstruct(opportunity) || ~isscalar(opportunity) || ...
        ~isfield(opportunity, 'expectedDetectionProbability') || ...
        ~isfinite(opportunity.expectedDetectionProbability)
    error('LmbLocalEvidence:InvalidOpportunity', ...
        'A finite expected detection opportunity is required.');
end
if ~isscalar(isScheduledSample) || ...
        ~(islogical(isScheduledSample) || ismember(isScheduledSample, [0, 1]))
    error('LmbLocalEvidence:InvalidScheduleFlag', ...
        'isScheduledSample must be a logical scalar.');
end

thresholds = resolveThresholds(options);
predictedExistence = predictedObject.r;
expectedDetection = clamp01( ...
    opportunity.expectedDetectionProbability);
absoluteDecrease = max(predictedExistence - updatedExistence, 0);
relativeDecrease = absoluteDecrease / max(predictedExistence, eps);

isPositiveSupport = detectionAssociationMass >= ...
    thresholds.positiveAssociationMass;
hasExistenceDecrease = ...
    absoluteDecrease >= thresholds.minimumAbsoluteExistenceDecrease && ...
    relativeDecrease >= thresholds.minimumRelativeExistenceDecrease;
isCredibleNegative = logical(isScheduledSample) && ...
    ~isPositiveSupport && ...
    expectedDetection >= thresholds.credibleOpportunity && ...
    hasExistenceDecrease;
isUnsupportedAbsence = ~isPositiveSupport && ...
    expectedDetection <= thresholds.unsupportedOpportunity;
isAmbiguous = ~(isPositiveSupport || isCredibleNegative || ...
    isUnsupportedAbsence);

if isPositiveSupport
    evidenceType = 'positive-support';
elseif isCredibleNegative
    evidenceType = 'credible-negative';
elseif isUnsupportedAbsence
    evidenceType = 'unsupported-absence';
else
    evidenceType = 'ambiguous';
end

evidence = struct();
evidence.contractVersion = 'lmb-label-local-evidence-v1';
evidence.type = evidenceType;
evidence.isPositiveSupport = isPositiveSupport;
evidence.isCredibleNegative = isCredibleNegative;
evidence.isUnsupportedAbsence = isUnsupportedAbsence;
evidence.isAmbiguous = isAmbiguous;
evidence.isAdmissibleToSafeReference = ...
    isPositiveSupport || isCredibleNegative;
evidence.predictedExistence = predictedExistence;
evidence.updatedExistence = updatedExistence;
evidence.absoluteExistenceDecrease = absoluteDecrease;
evidence.relativeExistenceDecrease = relativeDecrease;
evidence.detectionAssociationMass = detectionAssociationMass;
evidence.expectedDetectionProbability = expectedDetection;
evidence.isScheduledSample = logical(isScheduledSample);
evidence.thresholds = thresholds;
evidence.usesTargetTruth = false;
evidence.usesFutureMeasurements = false;
end

function thresholds = resolveThresholds(options)
thresholds = struct();
thresholds.positiveAssociationMass = validateUnitThreshold( ...
    options, 'positiveAssociationMass', 0.20);
thresholds.credibleOpportunity = validateUnitThreshold( ...
    options, 'credibleOpportunity', 0.50);
thresholds.unsupportedOpportunity = validateUnitThreshold( ...
    options, 'unsupportedOpportunity', 0.10);
thresholds.minimumAbsoluteExistenceDecrease = validateUnitThreshold( ...
    options, 'minimumAbsoluteExistenceDecrease', 0.01);
thresholds.minimumRelativeExistenceDecrease = validateUnitThreshold( ...
    options, 'minimumRelativeExistenceDecrease', 0.05);
if thresholds.unsupportedOpportunity >= thresholds.credibleOpportunity
    error('LmbLocalEvidence:InvalidThresholdOrder', ...
        'unsupportedOpportunity must be below credibleOpportunity.');
end
end

function value = validateUnitThreshold(options, fieldName, defaultValue)
value = getField(options, fieldName, defaultValue);
if ~isscalar(value) || ~isfinite(value) || value < 0 || value > 1
    error('LmbLocalEvidence:InvalidThreshold', ...
        '%s must be a finite scalar in [0, 1].', fieldName);
end
end

function value = getField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end

function value = clamp01(value)
value = min(max(value, 0), 1);
end
