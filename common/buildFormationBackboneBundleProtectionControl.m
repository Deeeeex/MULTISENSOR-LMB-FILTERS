function control = buildFormationBackboneBundleProtectionControl( ...
        context, options)
% BUILDFORMATIONBACKBONEBUNDLEPROTECTIONCONTROL Causal v38 base controller.
%
% Each registered formation-backbone input bundle is suppressed
% independently.  The
% resulting reference-relative expected-cardinality change is named a
% protection score: it measures whether withholding that bundle prevents
% existence-mass dilution under the current posterior and delivery model.
% It is not a generic information value or a correctness claim.

if nargin < 2 || isempty(options)
    options = struct();
end
policy = resolvePolicy(options);
nodeCount = numel(context.localPosteriorBySensor);
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
groups = unique(groupIds, 'stable');
formationCount = numel(groups);
if nodeCount < 2 || numel(groupIds) ~= nodeCount || ...
        formationCount < 2 || ...
        formationCount > policy.maximumFormationCount || ...
        ~isfield(context, 'previousAdjacencyHistory') || ...
        size(context.previousAdjacencyHistory, 1) ~= nodeCount || ...
        size(context.previousAdjacencyHistory, 2) ~= nodeCount || ...
        size(context.previousAdjacencyHistory, 3) < 1
    error('FormationBundleProtection:InvalidControlContext', ...
        'Formation input-bundle protection context is invalid.');
end

policyOptions = struct('policyConfig', policy);
[referenceAdjacency, referenceDetails] = ...
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, zeros(1, 0), policyOptions);
referenceWeights = referenceDetails.fusionWeightMatrix;
referenceScore = scoreFormationBackboneBundleRoute( ...
    context, referenceAdjacency, referenceWeights, ...
    [], groupIds, policy);
referenceDistributions = ...
    referenceScore.networkDetails.receiverDistributions;
referenceExpectedCardinality = ...
    referenceScore.retentionDetails.expectedReferenceCardinality;
if ~referenceScore.safe
    error('FormationBundleProtection:InvalidReference', ...
        'The registered reference route failed its safety gate.');
end

previouslySuspended = detectPreviousSuspensions( ...
    context, groupIds, groups);
thresholdByFormation = ...
    policy.protectionScoreOnFraction * ones(1, formationCount);
thresholdByFormation(previouslySuspended) = ...
    policy.protectionScoreOffFraction;
protectionScore = nan(1, formationCount);
referenceFormationExpectedCardinality = nan(1, formationCount);
singleSafe = false(1, formationCount);
singleAvailable = false(1, formationCount);
singleScores = cell(1, formationCount);
singleFailureIdentifier = repmat({''}, 1, formationCount);
singleFailureMessage = repmat({''}, 1, formationCount);
for formationIdx = 1:formationCount
    formationId = groups(formationIdx);
    try
        [adjacency, details] = ...
            selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
                context, formationId, policyOptions);
        score = scoreFormationBackboneBundleRoute( ...
            context, adjacency, details.fusionWeightMatrix, ...
            referenceDistributions, groupIds, policy);
        score.policyDetails = details;
        singleScores{formationIdx} = score;
        singleAvailable(formationIdx) = true;
        singleSafe(formationIdx) = score.safe && ...
            all(details.rollingB3SensorPass) && ...
            all(details.rollingB3FormationPass);
        members = groupIds == formationId;
        referenceFormationCardinality = mean( ...
            referenceExpectedCardinality(members));
        referenceFormationExpectedCardinality(formationIdx) = ...
            referenceFormationCardinality;
        protectionScore(formationIdx) = ...
            score.retentionDetails. ...
                formationMeanCardinalityChange(formationIdx) / max( ...
                    referenceFormationCardinality, 1);
    catch errorInfo
        if ~isExpectedUnavailable(errorInfo)
            rethrow(errorInfo);
        end
        singleFailureIdentifier{formationIdx} = ...
            getErrorField(errorInfo, 'identifier');
        singleFailureMessage{formationIdx} = ...
            getErrorField(errorInfo, 'message');
    end
end

cardinalityEvidenceMask = ...
    referenceFormationExpectedCardinality >= ...
        policy.minimumReferenceFormationExpectedCardinality;
requestedMask = singleAvailable & singleSafe & ...
    cardinalityEvidenceMask & ...
    protectionScore >= thresholdByFormation - 1e-12;
requestedFormationIds = groups(requestedMask);
selectedFormationIds = requestedFormationIds;
projectionRemovalOrder = zeros(1, 0);
selectedAdjacency = referenceAdjacency;
selectedWeights = referenceWeights;
selectedPolicyDetails = referenceDetails;
selectedScore = referenceScore;
jointEvaluationCount = 0;
while ~isempty(selectedFormationIds)
    jointEvaluationCount = jointEvaluationCount + 1;
    try
        [candidateAdjacency, candidateDetails] = ...
            selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
                context, selectedFormationIds, policyOptions);
        candidateScore = scoreFormationBackboneBundleRoute( ...
            context, candidateAdjacency, ...
            candidateDetails.fusionWeightMatrix, ...
            referenceDistributions, groupIds, policy);
        candidateSafe = candidateScore.safe && ...
            all(candidateDetails.rollingB3SensorPass) && ...
            all(candidateDetails.rollingB3FormationPass);
        if candidateSafe
            selectedAdjacency = candidateAdjacency;
            selectedWeights = candidateDetails.fusionWeightMatrix;
            selectedPolicyDetails = candidateDetails;
            selectedScore = candidateScore;
            break;
        end
    catch errorInfo
        if ~isExpectedUnavailable(errorInfo)
            rethrow(errorInfo);
        end
    end
    indices = zeros(1, numel(selectedFormationIds));
    for selectedIdx = 1:numel(selectedFormationIds)
        indices(selectedIdx) = find( ...
            groups == selectedFormationIds(selectedIdx), 1);
    end
    [~, removeLocalIdx] = min(protectionScore(indices));
    projectionRemovalOrder(end + 1) = ...
        selectedFormationIds(removeLocalIdx); %#ok<AGROW>
    selectedFormationIds(removeLocalIdx) = [];
end
if isempty(selectedFormationIds)
    selectedAdjacency = referenceAdjacency;
    selectedWeights = referenceWeights;
    selectedPolicyDetails = referenceDetails;
    selectedScore = referenceScore;
end

selectedMessageCount = nnz(selectedAdjacency);
referenceMessageCount = nnz(referenceAdjacency);
selectedWeightSupport = selectedAdjacency | logical(eye(nodeCount));
if ~selectedScore.safe || ...
        any(selectedAdjacency(:) & ...
            ~logical(context.physicalAdjacency(:))) || ...
        selectedMessageCount > referenceMessageCount || ...
        any(~isfinite(selectedWeights(:))) || ...
        any(selectedWeights(:) < -1e-12) || ...
        any(selectedWeights(:) > 1e-12 & ...
            ~selectedWeightSupport(:)) || ...
        any(abs(sum(selectedWeights, 2) - 1) > 1e-12)
    error('FormationBundleProtection:InvalidSelectedAction', ...
        'Selected input-bundle action violates a hard invariant.');
end

control = struct();
control.contractVersion = ...
    'formation-backbone-bundle-protection-control-v1';
control.policyConfig = policy;
control.policyConfigSha256 = policy.canonicalSha256;
control.nodeCount = nodeCount;
control.formationCount = formationCount;
control.groupIds = groupIds;
control.groups = groups;
control.referenceAdjacency = referenceAdjacency;
control.referenceFusionWeights = referenceWeights;
control.referencePolicyDetails = referenceDetails;
control.referenceScore = referenceScore;
control.previouslySuspendedMask = previouslySuspended;
control.protectionThresholdByFormation = thresholdByFormation;
control.formationProtectionScore = protectionScore;
control.referenceFormationExpectedCardinality = ...
    referenceFormationExpectedCardinality;
control.cardinalityEvidenceMask = cardinalityEvidenceMask;
control.singleActionAvailableMask = singleAvailable;
control.singleActionSafetyMask = singleSafe;
control.singleActionScores = singleScores;
control.singleFailureIdentifier = singleFailureIdentifier;
control.singleFailureMessage = singleFailureMessage;
control.requestedFormationIds = requestedFormationIds;
control.selectedFormationIds = selectedFormationIds;
control.projectionRemovalOrder = projectionRemovalOrder;
control.selectedAdjacency = selectedAdjacency;
control.selectedFusionWeights = selectedWeights;
control.selectedPolicyDetails = selectedPolicyDetails;
control.selectedScore = selectedScore;
control.selectedActionName = selectedPolicyDetails.actionName;
control.referenceFallbackUsed = isempty(selectedFormationIds);
control.referenceMessageCount = referenceMessageCount;
control.selectedMessageCount = selectedMessageCount;
control.messageSavingCount = ...
    referenceMessageCount - selectedMessageCount;
control.evaluatedRouteCount = 1 + formationCount + ...
    jointEvaluationCount;
if control.evaluatedRouteCount > ...
        policy.maximumControlRouteEvaluations
    error('FormationBundleProtection:EvaluationBudgetExceeded', ...
        'The base control exceeded its shared evaluation budget.');
end
control.safeSingleActionCount = nnz(singleSafe);
control.selectionUsesCurrentPosterior = true;
control.selectionUsesCurrentLinkReliability = true;
control.selectionUsesSelectedTopologyHistory = true;
control.hysteresisEnabled = ...
    policy.protectionScoreOffFraction < ...
        policy.protectionScoreOnFraction;
control.selectedRollingB3SensorPass = ...
    selectedPolicyDetails.rollingB3SensorPass;
control.selectedRollingB3FormationPass = ...
    selectedPolicyDetails.rollingB3FormationPass;
control.truthUsed = false;
control.futureOutcomeUsed = false;
end

function policy = resolvePolicy(options)
if ~isstruct(options) || ~isscalar(options) || ...
        any(~ismember(fieldnames(options), {'policyConfig'}))
    error('FormationBundleProtection:InvalidOptions', ...
        'Only the shared policyConfig may be supplied.');
end
if isfield(options, 'policyConfig')
    policy = options.policyConfig;
else
    policy = getFormationBackboneBundleStaggeredRecoveryPolicyConfig();
end
expected = getFormationBackboneBundleStaggeredRecoveryPolicyConfig();
if ~isstruct(policy) || ~isscalar(policy) || ...
        ~isfield(policy, 'canonicalSha256') || ...
        ~strcmp(policy.canonicalSha256, expected.canonicalSha256) || ...
        ~isequaln(policy, expected)
    error('FormationBundleProtection:InvalidPolicyConfig', ...
        'The shared v38 policy config changed.');
end
end

function suspended = detectPreviousSuspensions( ...
        context, groupIds, groups)
previous = logical(context.previousAdjacencyHistory(:, :, end));
suspended = false(1, numel(groups));
for formationIdx = 1:numel(groups)
    receivers = groupIds == groups(formationIdx);
    outsideSenders = groupIds ~= groups(formationIdx);
    suspended(formationIdx) = ...
        ~any(any(previous(receivers, outsideSenders)));
end
end

function value = isExpectedUnavailable(errorInfo)
identifier = getErrorField(errorInfo, 'identifier');
value = strcmp(identifier, ...
        'FormationBundleSuspension:Infeasible') || ...
    strcmp(identifier, ...
        'FormationBundleSuspension:InvalidCandidate') || ...
    strcmp(identifier, 'FormationBackboneTour:NonphysicalTour');
end

function value = getErrorField(errorInfo, fieldName)
value = '';
try
    value = errorInfo.(fieldName);
catch
    value = '';
end
if isempty(value)
    value = '';
end
end
