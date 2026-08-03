function control = ...
    buildFormationBackboneBundleStaggeredRecoveryControl(context, options)
% BUILDFORMATIONBACKBONEBUNDLESTAGGEREDRECOVERYCONTROL Causal v38 release.

if nargin < 2 || isempty(options)
    options = struct();
end
base = buildFormationBackboneBundleProtectionControl(context, options);
policy = base.policyConfig;
nodeCount = base.nodeCount;
groups = base.groups;
groupIds = base.groupIds;

suspensionAge = detectSuspensionAge(context, groupIds, groups);
schedule = buildFormationProtectionCoverageReleaseSchedule( ...
    groups, base.selectedFormationIds, ...
    base.formationProtectionScore, suspensionAge, ...
    policy.minimumSuspensionAgeSteps, ...
    policy.minimumRetainedProtectionCoverageFraction);

candidateCount = schedule.candidateCount;
candidateAdjacency = cell(1, candidateCount);
candidateFusionWeights = cell(1, candidateCount);
candidatePolicyDetails = cell(1, candidateCount);
candidateScores = cell(1, candidateCount);
candidateAvailableMask = false(1, candidateCount);
candidateSafetyMask = false(1, candidateCount);
candidateEligibilityMask = false(1, candidateCount);
candidateDisagreementImprovementFraction = nan(1, candidateCount);
candidateMessageSavingCount = nan(1, candidateCount);
candidateFailureIdentifier = repmat({''}, 1, candidateCount);
candidateFailureMessage = repmat({''}, 1, candidateCount);
referenceDistributions = ...
    base.referenceScore.networkDetails.receiverDistributions;
incumbentRisk = base.selectedScore.networkRisk;
policyOptions = struct('policyConfig', policy);
for candidateIdx = 1:candidateCount
    retained = schedule. ...
        candidateRetainedFormationIds{candidateIdx};
    try
        [adjacency, details] = ...
            selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
                context, retained, policyOptions);
        score = scoreFormationBackboneBundleRoute( ...
            context, adjacency, details.fusionWeightMatrix, ...
            referenceDistributions, groupIds, policy);
        improvement = relativeImprovement( ...
            incumbentRisk, score.networkRisk);
        messageSaving = details.messageSavingCount;
        safe = score.safe && ...
            all(details.rollingB3SensorPass) && ...
            all(details.rollingB3FormationPass);
        eligible = safe && ...
            messageSaving >= policy.minimumMessageSavingCount && ...
            improvement >= policy. ...
                minimumIncumbentDisagreementImprovementFraction - 1e-12;
        candidateAdjacency{candidateIdx} = adjacency;
        candidateFusionWeights{candidateIdx} = ...
            details.fusionWeightMatrix;
        candidatePolicyDetails{candidateIdx} = details;
        candidateScores{candidateIdx} = score;
        candidateAvailableMask(candidateIdx) = true;
        candidateSafetyMask(candidateIdx) = safe;
        candidateEligibilityMask(candidateIdx) = eligible;
        candidateDisagreementImprovementFraction(candidateIdx) = ...
            improvement;
        candidateMessageSavingCount(candidateIdx) = messageSaving;
    catch errorInfo
        if ~isExpectedUnavailable(errorInfo)
            rethrow(errorInfo);
        end
        candidateFailureIdentifier{candidateIdx} = ...
            getErrorField(errorInfo, 'identifier');
        candidateFailureMessage{candidateIdx} = ...
            getErrorField(errorInfo, 'message');
    end
end

selectedCandidateIndex = find(candidateEligibilityMask, 1, 'first');
selectedAdjacency = base.selectedAdjacency;
selectedFusionWeights = base.selectedFusionWeights;
selectedPolicyDetails = base.selectedPolicyDetails;
selectedScore = base.selectedScore;
selectedFormationIds = base.selectedFormationIds;
releasedFormationIds = zeros(1, 0);
selectedProtectionCoverageFraction = 1;
selectedDisagreementImprovementFraction = 0;
staggeredReleaseUsed = ~isempty(selectedCandidateIndex);
if staggeredReleaseUsed
    selectedAdjacency = candidateAdjacency{selectedCandidateIndex};
    selectedFusionWeights = ...
        candidateFusionWeights{selectedCandidateIndex};
    selectedPolicyDetails = ...
        candidatePolicyDetails{selectedCandidateIndex};
    selectedScore = candidateScores{selectedCandidateIndex};
    selectedFormationIds = schedule. ...
        candidateRetainedFormationIds{selectedCandidateIndex};
    releasedFormationIds = schedule. ...
        candidateReleasedFormationIds{selectedCandidateIndex};
    selectedProtectionCoverageFraction = schedule. ...
        candidateRetainedProtectionCoverage(selectedCandidateIndex);
    selectedDisagreementImprovementFraction = ...
        candidateDisagreementImprovementFraction(selectedCandidateIndex);
end

selectedWeightSupport = selectedAdjacency | logical(eye(nodeCount));
selectedMessageCount = nnz(selectedAdjacency);
if ~selectedScore.safe || ...
        any(selectedAdjacency(:) & ...
            ~logical(context.physicalAdjacency(:))) || ...
        selectedMessageCount > base.referenceMessageCount || ...
        any(~isfinite(selectedFusionWeights(:))) || ...
        any(selectedFusionWeights(:) < -1e-12) || ...
        any(selectedFusionWeights(:) > 1e-12 & ...
            ~selectedWeightSupport(:)) || ...
        any(abs(sum(selectedFusionWeights, 2) - 1) > 1e-12) || ...
        (staggeredReleaseUsed && ...
         (selectedProtectionCoverageFraction < policy. ...
            minimumRetainedProtectionCoverageFraction - 1e-12 || ...
          selectedDisagreementImprovementFraction < policy. ...
            minimumIncumbentDisagreementImprovementFraction - 1e-12))
    error('FormationBundleRecovery:InvalidSelection', ...
        'Selected staggered-recovery route violates a hard invariant.');
end

evaluatedRouteCount = base.evaluatedRouteCount + candidateCount;
if evaluatedRouteCount > policy.maximumControlRouteEvaluations
    error('FormationBundleRecovery:EvaluationBudgetExceeded', ...
        'Staggered recovery exceeded its shared evaluation budget.');
end

control = struct();
control.contractVersion = ...
    'formation-backbone-bundle-staggered-recovery-control-v1';
control.policyConfig = policy;
control.policyConfigSha256 = policy.canonicalSha256;
control.nodeCount = nodeCount;
control.formationCount = base.formationCount;
control.groupIds = groupIds;
control.groups = groups;
control.baseControl = base;
control.referenceAdjacency = base.referenceAdjacency;
control.referenceFusionWeights = base.referenceFusionWeights;
control.referencePolicyDetails = base.referencePolicyDetails;
control.referenceScore = base.referenceScore;
control.incumbentFormationIds = base.selectedFormationIds;
control.incumbentAdjacency = base.selectedAdjacency;
control.incumbentFusionWeights = base.selectedFusionWeights;
control.incumbentPolicyDetails = base.selectedPolicyDetails;
control.incumbentScore = base.selectedScore;
control.suspensionAgeByFormation = suspensionAge;
control.releaseSchedule = schedule;
control.candidateAdjacency = candidateAdjacency;
control.candidateFusionWeights = candidateFusionWeights;
control.candidatePolicyDetails = candidatePolicyDetails;
control.candidateScores = candidateScores;
control.candidateAvailableMask = candidateAvailableMask;
control.candidateSafetyMask = candidateSafetyMask;
control.candidateEligibilityMask = candidateEligibilityMask;
control.candidateDisagreementImprovementFraction = ...
    candidateDisagreementImprovementFraction;
control.candidateMessageSavingCount = candidateMessageSavingCount;
control.candidateFailureIdentifier = candidateFailureIdentifier;
control.candidateFailureMessage = candidateFailureMessage;
control.selectedCandidateIndex = selectedCandidateIndex;
control.selectedAdjacency = selectedAdjacency;
control.selectedFusionWeights = selectedFusionWeights;
control.selectedPolicyDetails = selectedPolicyDetails;
control.selectedScore = selectedScore;
control.selectedFormationIds = selectedFormationIds;
control.releasedFormationIds = releasedFormationIds;
control.selectedProtectionCoverageFraction = ...
    selectedProtectionCoverageFraction;
control.selectedDisagreementImprovementFraction = ...
    selectedDisagreementImprovementFraction;
control.selectedActionName = selectedPolicyDetails.actionName;
control.staggeredReleaseUsed = staggeredReleaseUsed;
control.baseFallbackUsed = ~staggeredReleaseUsed;
control.referenceFallbackUsed = isempty(selectedFormationIds);
control.referenceMessageCount = base.referenceMessageCount;
control.selectedMessageCount = selectedMessageCount;
control.messageSavingCount = ...
    base.referenceMessageCount - selectedMessageCount;
control.evaluatedRouteCount = evaluatedRouteCount;
control.safeCandidateCount = nnz(candidateSafetyMask);
control.eligibleCandidateCount = nnz(candidateEligibilityMask);
control.selectedRollingB3SensorPass = ...
    selectedPolicyDetails.rollingB3SensorPass;
control.selectedRollingB3FormationPass = ...
    selectedPolicyDetails.rollingB3FormationPass;
control.selectionUsesCurrentPosterior = true;
control.selectionUsesCurrentLinkReliability = true;
control.selectionUsesSelectedTopologyHistory = true;
control.truthUsed = false;
control.futureOutcomeUsed = false;
end

function age = detectSuspensionAge(context, groupIds, groups)
history = logical(context.previousAdjacencyHistory);
age = zeros(1, numel(groups));
for formationIdx = 1:numel(groups)
    receivers = groupIds == groups(formationIdx);
    outsideSenders = groupIds ~= groups(formationIdx);
    for historyIdx = size(history, 3):-1:1
        page = history(:, :, historyIdx);
        if any(any(page(receivers, outsideSenders)))
            break;
        end
        age(formationIdx) = age(formationIdx) + 1;
    end
end
end

function value = relativeImprovement(referenceRisk, candidateRisk)
value = (referenceRisk - candidateRisk) / max(referenceRisk, eps);
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
