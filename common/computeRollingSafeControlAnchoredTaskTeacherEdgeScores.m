function [scores, details] = ...
    computeRollingSafeControlAnchoredTaskTeacherEdgeScores( ...
        context, receiverIndices, senderIndices, ...
        cycleSources, options)
% COMPUTEROLLINGSAFECONTROLANCHOREDTASKTEACHEREDGESCORES
% Privileged safe-improvement labels relative to a scheduled control.
%
% The scheduled control supplies one reference source per receiver under
% the same physical graph, history and rolling-B3 contract. Candidate
% scores measure expected task-risk improvement over that reference.
% Per-receiver reference risks are also returned as hard upper bounds for
% the joint projector. Thus a receiver whose cycle fallback is worse than
% the control must retain the control edge or select a no-worse substitute.
%
% Truth enters through computeRollingSafeTaskTeacherEdgeScores. This helper
% is an offline attainability diagnostic, not a deployable policy.

if nargin < 5 || isempty(options)
    options = struct();
end
sourceWeight = getField(options, 'sourceWeight', 0.70);
anchorMode = getField(options, ...
    'anchorMode', 'scheduled-burst');
anchorOptions = getField(options, ...
    'anchorPolicyOptions', struct());
anchorOptions.sourceWeight = sourceWeight;
anchorOptions.payloadToleranceFraction = getField(options, ...
    'payloadToleranceFraction', inf);

[~, anchorDetails] = selectRollingSafeRoutingPolicy( ...
    context, anchorMode, anchorOptions);
requireNominalAnchor = logical(getField( ...
    options, 'requireNominalAnchor', true));
if requireNominalAnchor && ( ...
        ~anchorDetails.nominalProjectionFeasible || ...
        anchorDetails.repairTriggered || ...
        anchorDetails.payloadEmergencyUsed || ...
        ~anchorDetails.successorScheduleConstrained)
    error(['The control-anchored teacher requires an exactly executed ', ...
        'nominal scheduled reference with no repair or payload ', ...
        'emergency.']);
end
anchorSources = reshape( ...
    anchorDetails.selectedSourcesByReceiver, 1, []);
nodeCount = numel(cycleSources);
if numel(anchorSources) ~= nodeCount
    error('Control anchor must provide one source per receiver.');
end

teacherOptions = getField(options, ...
    'teacherOptions', struct());
teacherOptions.sourceWeight = sourceWeight;
[~, details] = computeRollingSafeTaskTeacherEdgeScores( ...
    context, receiverIndices, senderIndices, ...
    cycleSources, teacherOptions);

gainByEdge = details.teacherDetails. ...
    firstStepExpectedGainByWeight;
weightGrid = reshape(details.teacherDetails. ...
    sourceWeightGrid, 1, []);
[weightDifference, weightIdx] = ...
    min(abs(weightGrid - sourceWeight));
if isempty(weightIdx) || weightDifference > 1e-12
    error('Control anchor risk uses an unavailable source weight.');
end
anchorGain = nan(1, nodeCount);
for receiverIdx = 1:nodeCount
    anchorGain(receiverIdx) = gainByEdge( ...
        receiverIdx, anchorSources(receiverIdx), weightIdx);
end
if any(~isfinite(anchorGain))
    error(['The scheduled control selected an edge without a finite ', ...
        'teacher risk evaluation.']);
end
anchorRisk = details.teacherDetails.nodeRiskBefore - ...
    anchorGain;
riskBoundMode = lower(strrep(char(getField( ...
    options, 'riskBoundMode', 'per-receiver')), '_', '-'));
switch riskBoundMode
    case 'per-receiver'
        riskUpperBound = anchorRisk;
    case 'global-tail'
        riskUpperBound = ...
            max(anchorRisk) * ones(size(anchorRisk));
    otherwise
        error('Unknown control-anchor riskBoundMode: %s', ...
            riskBoundMode);
end
candidateRisk = reshape( ...
    details.candidateExpectedRisk, [], 1);
receiverIndices = reshape(receiverIndices, [], 1);
controlAnchoredResidualScores = ...
    -inf(size(candidateRisk));
for exampleIdx = 1:numel(candidateRisk)
    receiverIdx = receiverIndices(exampleIdx);
    if isfinite(candidateRisk(exampleIdx))
        controlAnchoredResidualScores(exampleIdx) = ...
            anchorRisk(receiverIdx) - ...
            candidateRisk(exampleIdx);
    end
end
% The final route falls back to the cycle source whenever no cross edge is
% selected. Therefore the additive MILP coefficient must be
% cycleRisk-candidateRisk. The anchor contributes receiver-specific risk
% constraints, but subtracting anchorRisk in the objective would omit the
% selected-versus-cycle term for receivers whose anchor itself is cross
% formation.
scores = reshape(details.unweightedResidualScores, [], 1);

details.controlAnchoredResidualScores = ...
    controlAnchoredResidualScores;
details.projectionObjectiveResidualScores = scores;
details.riskUpperBoundByReceiver = riskUpperBound;
details.riskReferenceSourcesByReceiver = anchorSources;
details.riskReferenceExpectedGainByReceiver = anchorGain;
details.riskReferenceExpectedRiskByReceiver = anchorRisk;
details.riskReferenceMode = ...
    ['rolling-safe-', lower(strrep( ...
        char(anchorMode), '_', '-'))];
details.riskBoundMode = riskBoundMode;
details.riskReferencePolicyDetails = anchorDetails;
details.nominalRiskReferenceRequired = ...
    requireNominalAnchor;
details.cycleSourcesByReceiver = ...
    reshape(cycleSources, 1, []);
details.truthUsed = true;
details.posteriorUsed = true;
details.currentLinkReliabilityUsed = true;
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
