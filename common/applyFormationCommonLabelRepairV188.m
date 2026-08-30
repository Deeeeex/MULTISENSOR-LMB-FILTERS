function [posteriorBySensor, details] = ...
        applyFormationCommonLabelRepairV188( ...
            posteriorBySensor, proposals, selectedProposalIndices)
% APPLYFORMATIONCOMMONLABELREPAIRV188 Apply exact complete-label payloads.

sensorCount = numel(posteriorBySensor);
if nargin < 3 || isempty(selectedProposalIndices)
    selectedProposalIndices = zeros(1, 0);
end
if ~iscell(posteriorBySensor) || ~isstruct(proposals)
    error('FormationRepairApplyV188:InvalidInput', ...
        'The posterior bank or proposal set is malformed.');
end
if islogical(selectedProposalIndices)
    if numel(selectedProposalIndices) ~= numel(proposals)
        error('FormationRepairApplyV188:SelectionMismatch', ...
            'A logical selection must align with the proposal set.');
    end
    selectedProposalIndices = find(selectedProposalIndices);
end
selectedProposalIndices = reshape(selectedProposalIndices, 1, []);
if any(~isfinite(selectedProposalIndices)) || ...
        any(selectedProposalIndices ~= round(selectedProposalIndices)) || ...
        any(selectedProposalIndices < 1) || ...
        any(selectedProposalIndices > numel(proposals)) || ...
        numel(unique(selectedProposalIndices)) ~= ...
            numel(selectedProposalIndices)
    error('FormationRepairApplyV188:InvalidSelection', ...
        'The selected proposal indices are invalid.');
end

appliedReceiverIds = zeros(1, 0);
appliedFormationIds = zeros(1, 0);
appliedLabels = zeros(2, 0);
appliedSources = zeros(1, 0);
appliedKeys = zeros(0, 3);
for proposalIdx = selectedProposalIndices
    proposal = proposals(proposalIdx);
    validateProposal(proposal, sensorCount);
    for receiverIdx = reshape(proposal.receiverIds, 1, [])
        key = [receiverIdx, reshape(proposal.label, 1, 2)];
        if ~isempty(appliedKeys) && ismember(key, appliedKeys, 'rows')
            error('FormationRepairApplyV188:DuplicateReceiverLabel', ...
                'A receiver label cannot be repaired twice on one page.');
        end
        posteriorBySensor{receiverIdx} = replaceLabelObject( ...
            posteriorBySensor{receiverIdx}, proposal.sourceObject);
        appliedKeys(end + 1, :) = key; %#ok<AGROW>
        appliedReceiverIds(end + 1) = receiverIdx; %#ok<AGROW>
        appliedFormationIds(end + 1) = proposal.formationId; %#ok<AGROW>
        appliedLabels(:, end + 1) = proposal.label; %#ok<AGROW>
        appliedSources(end + 1) = proposal.sourceId; %#ok<AGROW>
    end
end

details = struct();
details.contractVersion = ...
    'formation-common-label-repair-apply-v188-v1';
details.selectedProposalIndices = selectedProposalIndices;
details.appliedActionCount = numel(selectedProposalIndices);
details.appliedReceiverCount = numel(appliedReceiverIds);
details.appliedReceiverIds = appliedReceiverIds;
details.appliedFormationIds = appliedFormationIds;
details.appliedLabels = appliedLabels;
details.appliedSources = appliedSources;
details.completePayloadApplied = true;
details.momentMatchedPayloadApplied = false;
details.truthUsed = false;
details.futureInformationUsed = false;
end

function validateProposal(proposal, sensorCount)
required = {'formationId', 'receiverIds', 'sourceId', 'label', ...
    'sourceObject', 'safetyPassed', 'rollingConnectivityPassed'};
if any(~isfield(proposal, required)) || ...
        ~isscalar(proposal.formationId) || ...
        proposal.formationId < 1 || ...
        proposal.formationId ~= round(proposal.formationId) || ...
        ~isscalar(proposal.sourceId) || proposal.sourceId < 1 || ...
        proposal.sourceId > sensorCount || ...
        proposal.sourceId ~= round(proposal.sourceId) || ...
        ~isequal(size(proposal.label), [2, 1]) || ...
        any(~isfinite(proposal.label)) || ...
        ~isscalar(proposal.sourceObject) || ...
        ~isstruct(proposal.sourceObject) || ...
        ~validTrueBoolean(proposal.safetyPassed) || ...
        ~validTrueBoolean(proposal.rollingConnectivityPassed)
    error('FormationRepairApplyV188:UnsafeProposal', ...
        'Only a fully projected executable proposal may be applied.');
end
receivers = reshape(proposal.receiverIds, 1, []);
if isempty(receivers) || any(~isfinite(receivers)) || ...
        any(receivers ~= round(receivers)) || any(receivers < 1) || ...
        any(receivers > sensorCount) || ...
        numel(unique(receivers)) ~= numel(receivers) || ...
        proposal.sourceObject.birthTime ~= proposal.label(1) || ...
        proposal.sourceObject.birthLocation ~= proposal.label(2)
    error('FormationRepairApplyV188:ProposalContractDrift', ...
        'The executable proposal routing key is malformed.');
end
end

function posterior = replaceLabelObject(posterior, object)
posterior = reshape(posterior, 1, []);
if isempty(posterior)
    posterior = reshape(object, 1, 1);
    return;
end
label = [object.birthTime; object.birthLocation];
idx = 0;
for objectIdx = 1:numel(posterior)
    if posterior(objectIdx).birthTime == label(1) && ...
            posterior(objectIdx).birthLocation == label(2)
        idx = objectIdx;
        break;
    end
end
if idx == 0
    posterior(end + 1) = object;
else
    posterior(idx) = object;
end
end

function valid = validTrueBoolean(value)
valid = isscalar(value) && ...
    (islogical(value) || (isnumeric(value) && isfinite(value) && ...
    (value == 0 || value == 1))) && logical(value);
end
