function restricted = ...
        restrictBudgetedLocalGatewayTeacherCandidateBankV255( ...
            bank, protocol, options)
% RESTRICTBUDGETEDLOCALGATEWAYTEACHERCANDIDATEBANKV255 Local one-arc bank.

if nargin < 2 || isempty(protocol)
    protocol = getBudgetedLocalGatewayTeacherV255Protocol();
end
if nargin < 3 || isempty(options)
    options = struct();
end
requireEveryDirectedSlot = logical(getField( ...
    options, 'requireEveryDirectedSlot', true));
minimumCandidateCount = getField( ...
    options, 'minimumCandidateCount', protocol.minimumCandidateCount);
requiredBankFields = {'contractVersion', 'referenceCandidateIndex', ...
    'candidateCount', 'candidates', 'formationPhysicalUids', ...
    'expectedMessageCount', 'hardGatePassed'};
valid = isstruct(bank) && isscalar(bank) && ...
    all(isfield(bank, requiredBankFields)) && ...
    strcmp(bank.contractVersion, ...
        'causal-gateway-embedding-v250-bank-v1') && ...
    bank.referenceCandidateIndex == 1 && ...
    bank.candidateCount == numel(bank.candidates) && ...
    bank.hardGatePassed && ...
    isstruct(protocol) && isscalar(protocol) && ...
    isfield(protocol, 'maximumCandidatesPerDirectedSlot') && ...
    isfield(protocol, 'maximumRestrictedCandidateCount') && ...
    isfield(protocol, 'minimumCandidateCount') && ...
    islogical(requireEveryDirectedSlot) && ...
    isscalar(requireEveryDirectedSlot) && ...
    isscalar(minimumCandidateCount) && ...
    minimumCandidateCount >= 2 && ...
    minimumCandidateCount == round(minimumCandidateCount) && ...
    minimumCandidateCount <= protocol.maximumRestrictedCandidateCount;
if ~valid
    error('BudgetedLocalGatewayTeacherV255:InvalidSourceBank', ...
        'The V255 restriction requires a valid V250 candidate bank.');
end

reference = normalizeAssignment( ...
    bank.candidates(bank.referenceCandidateIndex).gatewayAssignment);
slotCount = size(reference, 1);
records = zeros(0, 3);
for candidateIdx = 1:bank.candidateCount
    candidate = bank.candidates(candidateIdx);
    if ~strcmp(candidate.candidateType, 'single-directed-arc')
        continue;
    end
    assignment = normalizeAssignment(candidate.gatewayAssignment);
    changed = find(any(assignment(:, 3:4) ~= reference(:, 3:4), 2));
    formationPairsPreserved = ...
        isequal(assignment(:, 1:2), reference(:, 1:2));
    if numel(changed) ~= 1 || ~formationPairsPreserved || ...
            candidate.sourceUid ~= changed || candidate.rank < 1
        error('BudgetedLocalGatewayTeacherV255:InvalidSingleArc', ...
            'A V250 single-arc candidate changed the wrong slot.');
    end
    records(end + 1, :) = ... %#ok<AGROW>
        [candidate.rank, candidate.sourceUid, candidateIdx];
end
if isempty(records)
    error('BudgetedLocalGatewayTeacherV255:MissingSingleArcBank', ...
        'The source bank contains no one-arc replacements.');
end

records = sortrows(records, [1, 2, 3]);
selectedSourceIndices = bank.referenceCandidateIndex;
slotCandidateCount = zeros(1, slotCount);
for recordIdx = 1:size(records, 1)
    slotIdx = records(recordIdx, 2);
    if slotIdx < 1 || slotIdx > slotCount || ...
            slotIdx ~= round(slotIdx)
        error('BudgetedLocalGatewayTeacherV255:InvalidSlot', ...
            'A one-arc candidate references an unknown directed slot.');
    end
    if slotCandidateCount(slotIdx) >= ...
            protocol.maximumCandidatesPerDirectedSlot
        continue;
    end
    selectedSourceIndices(end + 1) = ... %#ok<AGROW>
        records(recordIdx, 3);
    slotCandidateCount(slotIdx) = slotCandidateCount(slotIdx) + 1;
end

candidateCount = numel(selectedSourceIndices);
validCoverage = ~requireEveryDirectedSlot || ...
    all(slotCandidateCount >= 1);
validCount = candidateCount >= minimumCandidateCount && ...
    candidateCount <= protocol.maximumRestrictedCandidateCount && ...
    candidateCount <= 1 + ...
        protocol.maximumCandidatesPerDirectedSlot * slotCount;
if ~validCoverage || ~validCount
    error('BudgetedLocalGatewayTeacherV255:RestrictedBankCoverage', ...
        'The local bank lacks the registered action coverage or count.');
end

candidates = bank.candidates(selectedSourceIndices);
changedArcCount = zeros(1, candidateCount);
for candidateIdx = 1:candidateCount
    candidates(candidateIdx).candidateIndex = candidateIdx;
    assignment = normalizeAssignment(candidates(candidateIdx).gatewayAssignment);
    changedArcCount(candidateIdx) = sum(any( ...
        assignment(:, 3:4) ~= reference(:, 3:4), 2));
end
structuralGate = changedArcCount(1) == 0 && ...
    all(changedArcCount(2:end) == 1) && ...
    all([candidates.messageCount] == bank.expectedMessageCount) && ...
    all([candidates.stronglyConnected]) && ...
    all([candidates.physicallyFeasible]) && ...
    all([candidates.weightsValid]) && ...
    all([candidates.requestedGatewayAssignmentApplied]);
if ~structuralGate
    error('BudgetedLocalGatewayTeacherV255:RestrictedBankStructure', ...
        'A restricted V255 candidate violates the V242 topology contract.');
end

coverage = receiverCoverage(candidates, bank.formationPhysicalUids);
restricted = bank;
restricted.restrictionContractVersion = ...
    'budgeted-local-gateway-teacher-v255-bank-restriction-v1';
restricted.candidateBankMode = ...
    'v242-reference-plus-ranked-single-directed-arc';
restricted.sourceCandidateIndices = selectedSourceIndices;
restricted.candidateCount = candidateCount;
restricted.candidates = candidates;
restricted.receiverCoverageByFormation = coverage;
restricted.minimumReceiverCoverage = min(coverage);
restricted.directedFormationSlotCount = slotCount;
restricted.candidateCountByDirectedSlot = slotCandidateCount;
restricted.changedDirectedArcCountByCandidate = changedArcCount;
restricted.maximumCandidatesPerDirectedSlot = ...
    protocol.maximumCandidatesPerDirectedSlot;
restricted.requireEveryDirectedSlot = requireEveryDirectedSlot;
restricted.minimumRestrictedCandidateCount = minimumCandidateCount;
restricted.hardGatePassed = true;
restricted.truthUsed = false;
restricted.posteriorUsed = false;
restricted.measurementUsed = false;
restricted.futurePhysicalPageUsed = false;
restricted.trackingOutcomeUsed = false;
end

function coverage = receiverCoverage(candidates, formationUids)
coverage = zeros(1, numel(formationUids));
for formationIdx = 1:numel(formationUids)
    receiverUid = formationUids(formationIdx);
    represented = zeros(1, 0);
    for candidateIdx = 1:numel(candidates)
        assignment = candidates(candidateIdx).gatewayAssignment;
        represented = [represented, ... %#ok<AGROW>
            reshape(assignment(assignment(:, 2) == receiverUid, 4), 1, [])];
    end
    coverage(formationIdx) = numel(unique(represented));
end
end

function assignment = normalizeAssignment(assignment)
assignment = sortrows(assignment, [2, 1, 4, 3]);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
