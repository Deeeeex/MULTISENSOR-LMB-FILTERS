function test_formation_b4_v46_tracking_outcome_evaluator()
% The frozen evaluator must reject pooled rescue and malformed evidence.

evaluation = getFormationB4V46TrackingEvaluationRegistry();

developmentRecords = buildPassingRecords(evaluation, 'development');
development = evaluateFormationB4V46TrackingOutcomeMatrix( ...
    developmentRecords(end:-1:1), evaluation, 'development');
assert(development.development.passed);
assert(development.developmentAdvanceRulePassed);
assert(~development.confirmationRulePassed);
assert(isequal(development.orderedCaseOrdinals, sort( ...
    [developmentRecords.caseOrdinal])));

records = buildPassingRecords(evaluation, 'confirmation');
confirmation = evaluateFormationB4V46TrackingOutcomeMatrix( ...
    records, evaluation, 'confirmation');
reordered = evaluateFormationB4V46TrackingOutcomeMatrix( ...
    records(end:-1:1), evaluation, 'confirmation');
assert(confirmation.confirmationRulePassed);
assert(confirmation.confirmation.allSeedBlocksPassed);
assert(confirmation.confirmation.passingSeedBlockCount == 4);
assert(strcmp(confirmation.canonicalSha256, reordered.canonicalSha256));
byteSummary = findMetric(confirmation, 'attemptedByteSaving');
assert(max(abs(byteSummary.seedBlockValues - 0.36)) < 1e-12);
assert(abs(byteSummary.grandMean - 0.36) < 1e-12);
assert(abs(byteSummary.grandMedian - 0.36) < 1e-12);
assert(max(abs(byteSummary.observedSeedRange - [0.36, 0.36])) < 1e-12);
assert(max(abs(byteSummary.leaveOneSeedOutMeans - 0.36)) < 1e-12);
assert(max(abs(byteSummary.leaveOneSeedOutMeanRange - ...
    [0.36, 0.36])) < 1e-12);
assert(max(abs(byteSummary.scaleSeedValues(:) - 0.36)) < 1e-12);

onePairFails = records;
failedOrdinal = evaluation.confirmationAnalysis. ...
    primaryCaseOrdinalMatrix(1, 1);
failedIdx = find([onePairFails.caseOrdinal] == failedOrdinal);
onePairFails(failedIdx).candidate.fullHorizonPositionEospa = 16.0;
failed = evaluateFormationB4V46TrackingOutcomeMatrix( ...
    onePairFails, evaluation, 'confirmation');
assert(~failed.confirmation.seedBlocks(1).passed);
assert(all([failed.confirmation.seedBlocks(2:4).passed]));
assert(~failed.confirmationRulePassed);

stressCannotRescue = records;
stressOrdinal = evaluation.confirmationAnalysis. ...
    stressCaseOrdinalMatrix(1, 1);
stressIdx = find([stressCannotRescue.caseOrdinal] == stressOrdinal);
stressCannotRescue(stressIdx).runtimeAuditPassed = false;
stressCannotRescue(stressIdx).candidate = perfectCandidateSummary();
stressFailed = evaluateFormationB4V46TrackingOutcomeMatrix( ...
    stressCannotRescue, evaluation, 'confirmation');
assert(~stressFailed.confirmation.seedBlocks(1).stressCompletionPassed);
assert(~stressFailed.confirmationRulePassed);

stressUndefinedIsDescriptive = records;
stressUndefinedIsDescriptive(stressIdx).reference. ...
    meanConsensusPositionOspa = 0;
stressUndefinedIsDescriptive(stressIdx).candidate. ...
    meanConsensusPositionOspa = 0.1;
stressDescriptive = evaluateFormationB4V46TrackingOutcomeMatrix( ...
    stressUndefinedIsDescriptive, evaluation, 'confirmation');
assert(stressDescriptive.confirmation.seedBlocks(1).passed);
assert(~stressDescriptive.confirmation.seedBlocks(1). ...
    allStressChangesDefinedForDescription);
assert(stressDescriptive.confirmationRulePassed);

zeroReferenceHarm = records;
zeroIdx = find([zeroReferenceHarm.caseOrdinal] == failedOrdinal);
zeroReferenceHarm(zeroIdx).reference.meanConsensusPositionOspa = 0;
zeroReferenceHarm(zeroIdx).candidate.meanConsensusPositionOspa = 0.1;
zeroFailed = evaluateFormationB4V46TrackingOutcomeMatrix( ...
    zeroReferenceHarm, evaluation, 'confirmation');
assert(~zeroFailed.confirmation.seedBlocks(1).allChangesDefined);
assert(~zeroFailed.confirmationRulePassed);
consensusSummary = findMetric( ...
    zeroFailed, 'meanConsensusPositionOspaIncrease');
assert(isnan(consensusSummary.seedBlockValues(1)));
assert(isnan(consensusSummary.scaleSeedValues(1, 1)));
assert(isfinite(consensusSummary.scaleSeedValues(1, 2)));
assert(abs(consensusSummary.caseAbsoluteChanges(1, 1) - 0.1) < 1e-12);
zeroCaseResult = zeroFailed.caseResults( ...
    [zeroFailed.caseResults.caseOrdinal] == failedOrdinal);
assert(abs(zeroCaseResult.absoluteChanges. ...
    meanConsensusPositionOspaIncrease - 0.1) < 1e-12);
assert(~zeroFailed.confirmation.descriptive. ...
    allDescriptiveMetricsDefined);

zeroCommunicationReference = developmentRecords;
for recordIdx = 1:numel(zeroCommunicationReference)
    zeroCommunicationReference(recordIdx).reference. ...
        deliveredMessageCount = 0;
    zeroCommunicationReference(recordIdx).candidate. ...
        deliveredMessageCount = 0;
    zeroCommunicationReference(recordIdx).reference. ...
        deliveredPayloadScalars = 0;
    zeroCommunicationReference(recordIdx).candidate. ...
        deliveredPayloadScalars = 0;
    zeroCommunicationReference(recordIdx).reference. ...
        deliveredPayloadBytes = 0;
    zeroCommunicationReference(recordIdx).candidate. ...
        deliveredPayloadBytes = 0;
end
zeroCommunication = evaluateFormationB4V46TrackingOutcomeMatrix( ...
    zeroCommunicationReference, evaluation, 'development');
assert(~zeroCommunication.development.allPrimaryChangesDefined);
assert(~zeroCommunication.developmentAdvanceRulePassed);

assertThrows(@() evaluateFormationB4V46TrackingOutcomeMatrix( ...
    records(1:end-1), evaluation, 'confirmation'), ...
    'FormationB4V46TrackingOutcome:RecordCoverage');
duplicated = records;
duplicated(end).caseOrdinal = duplicated(1).caseOrdinal;
assertThrows(@() evaluateFormationB4V46TrackingOutcomeMatrix( ...
    duplicated, evaluation, 'confirmation'), ...
    'FormationB4V46TrackingOutcome:RecordCoverage');
nonfinite = records;
nonfinite(1).candidate.focusWindowPositionEospa = NaN;
assertThrows(@() evaluateFormationB4V46TrackingOutcomeMatrix( ...
    nonfinite, evaluation, 'confirmation'), ...
    'FormationB4V46TrackingOutcome:RawSummaryValue');
swapped = records;
swapped(1).referenceArmId = evaluation.candidateArmId;
assertThrows(@() evaluateFormationB4V46TrackingOutcomeMatrix( ...
    swapped, evaluation, 'confirmation'), ...
    'FormationB4V46TrackingOutcome:RecordSchema');
tampered = evaluation;
tampered.developmentThresholds. ...
    attemptedByteSavingPerPairMinimum = -999;
tampered.canonicalSha256 = computeCanonicalValueSha256( ...
    rmfield(tampered, 'canonicalSha256'));
assertThrows(@() evaluateFormationB4V46TrackingOutcomeMatrix( ...
    records, tampered, 'confirmation'), ...
    'FormationB4V46TrackingOutcome:EvaluationDrift');

assert(~confirmation.filterExecutionAuthorized);
assert(~confirmation.confirmationTrackingAuthorized);
assert(~confirmation.developmentAdvanceDecisionAuthorized);
assert(~confirmation.validationClaimAllowed);
assert(~confirmation.populationGeneralizationAllowed);
assert(~confirmation.statisticalSignificanceClaimAllowed);
fprintf('PASS: V46 tracking outcome evaluator tests\n');
end

function records = buildPassingRecords(evaluation, phase)
if strcmp(phase, 'development')
    ordinals = [evaluation.developmentPrimaryCaseOrdinals, ...
        evaluation.developmentStressCaseOrdinals];
else
    ordinals = [reshape(evaluation.confirmationAnalysis. ...
        primaryCaseOrdinalMatrix, 1, []), ...
        reshape(evaluation.confirmationAnalysis. ...
        stressCaseOrdinalMatrix, 1, [])];
end
ordinals = sort(ordinals);
template = emptyRecord();
records = repmat(template, 1, numel(ordinals));
for idx = 1:numel(ordinals)
    ordinal = ordinals(idx);
    value = template;
    value.contractVersion = ...
        evaluation.pairedSummaryRecordContractVersion;
    value.caseOrdinal = ordinal;
    value.caseId = evaluation.cases(ordinal).caseId;
    value.referenceArmId = evaluation.referenceArmId;
    value.candidateArmId = evaluation.candidateArmId;
    value.runtimeAuditPassed = true;
    value.reference = referenceSummary();
    value.candidate = candidateSummary();
    records(idx) = value;
end
end

function value = emptyRecord()
value = struct('contractVersion', '', 'caseOrdinal', NaN, ...
    'caseId', '', 'referenceArmId', '', 'candidateArmId', '', ...
    'runtimeAuditPassed', false, 'reference', referenceSummary(), ...
    'candidate', candidateSummary());
end

function value = referenceSummary()
value = struct( ...
    'attemptedMessageCount', 80.0, ...
    'deliveredMessageCount', 64.0, ...
    'attemptedPayloadScalars', 125.0, ...
    'deliveredPayloadScalars', 100.0, ...
    'attemptedPayloadBytes', 1000.0, ...
    'deliveredPayloadBytes', 800.0, ...
    'fullHorizonPositionEospa', 10.0, ...
    'focusWindowPositionEospa', 10.0, ...
    'worstSensorPositionEospa', 12.0, ...
    'meanAbsoluteCardinalityError', 1.0, ...
    'meanConsensusPositionOspa', 2.0, ...
    'terminalConsensusPositionOspa', 2.0, ...
    'repairPageCount', 0.0, 'firstRepairTime', 0.0);
end

function value = candidateSummary()
value = struct( ...
    'attemptedMessageCount', 50.0, ...
    'deliveredMessageCount', 40.0, ...
    'attemptedPayloadScalars', 80.0, ...
    'deliveredPayloadScalars', 64.0, ...
    'attemptedPayloadBytes', 640.0, ...
    'deliveredPayloadBytes', 512.0, ...
    'fullHorizonPositionEospa', 10.1, ...
    'focusWindowPositionEospa', 10.1, ...
    'worstSensorPositionEospa', 12.12, ...
    'meanAbsoluteCardinalityError', 1.01, ...
    'meanConsensusPositionOspa', 2.02, ...
    'terminalConsensusPositionOspa', 2.02, ...
    'repairPageCount', 2.0, 'firstRepairTime', 5.0);
end

function value = perfectCandidateSummary()
value = candidateSummary();
value.fullHorizonPositionEospa = 0;
value.focusWindowPositionEospa = 0;
value.worstSensorPositionEospa = 0;
value.meanAbsoluteCardinalityError = 0;
value.meanConsensusPositionOspa = 0;
value.terminalConsensusPositionOspa = 0;
end

function value = findMetric(result, id)
metrics = result.confirmation.descriptive.metrics;
match = find(strcmp({metrics.id}, id));
assert(numel(match) == 1);
value = metrics(match);
end

function assertThrows(callback, expectedIdentifier)
threw = false;
try
    callback();
catch err
    threw = true;
    assert(strcmp(err.identifier, expectedIdentifier));
end
assert(threw);
end
