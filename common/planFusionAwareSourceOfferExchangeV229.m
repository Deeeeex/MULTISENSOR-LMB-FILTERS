function plan = planFusionAwareSourceOfferExchangeV229( ...
        sourceSensorIds, candidatePayloadBytes, ...
        admissionNetSavingBytes, referencePageBytes, options)
% PLANFUSIONAWARESOURCEOFFEREXCHANGEV229 Bound remote-surprise discovery.
%
% Every eligible source receives one fixed-size solicitation and is charged
% for a response header plus the maximum number of compact offers, even if it
% has fewer useful labels.  A complete Bernoulli-GM payload is selected only
% after this worst-case control cost is paid.  Source ranking and delivery
% are deliberately outside this deterministic byte planner.

if nargin < 5 || isempty(options)
    options = struct();
end
protocol = getFusionAwareSourceOfferProtocolV229();
offer = protocol.offer;
maximumOffersPerSource = getField(options, ...
    'maximumOffersPerSource', offer.maximumOffersPerSource);
sourceSensorIds = unique(reshape(sourceSensorIds, 1, []), 'stable');
candidatePayloadBytes = reshape(candidatePayloadBytes, 1, []);
if isempty(sourceSensorIds) || ...
        any(~isfinite(sourceSensorIds)) || ...
        any(sourceSensorIds ~= round(sourceSensorIds)) || ...
        any(sourceSensorIds < 1) || ...
        isempty(candidatePayloadBytes) || ...
        any(~isfinite(candidatePayloadBytes)) || ...
        any(candidatePayloadBytes < 0) || ...
        any(candidatePayloadBytes ~= round(candidatePayloadBytes)) || ...
        ~validNonnegativeScalar(admissionNetSavingBytes) || ...
        ~validPositiveScalar(referencePageBytes) || ...
        ~isscalar(maximumOffersPerSource) || ...
        ~isfinite(maximumOffersPerSource) || ...
        maximumOffersPerSource < 1 || ...
        maximumOffersPerSource ~= round(maximumOffersPerSource) || ...
        maximumOffersPerSource > offer.maximumOffersPerSource
    error('FusionAwareSourceOfferV229:InvalidInput', ...
        'The source-offer credit request is malformed.');
end

sourceCount = numel(sourceSensorIds);
solicitationAttemptedBytes = ...
    sourceCount * offer.solicitationHeaderBytes;
responseBytesPerSource = offer.responseHeaderBytes + ...
    maximumOffersPerSource * offer.offerRecordBytes;
responseAttemptedBytes = sourceCount * responseBytesPerSource;
controlAttemptedBytes = ...
    solicitationAttemptedBytes + responseAttemptedBytes;
[creditAfterOffers, controlDecision] = ...
    preflightBudgetRecycledRepairSynopsisV188( ...
        [], admissionNetSavingBytes, controlAttemptedBytes, ...
        referencePageBytes);

feasibleMask = controlDecision.lightSynopsisAuthorized & ...
    (candidatePayloadBytes <= ...
        creditAfterOffers.spendableCreditBytes + 1e-9);
netSavingBytes = admissionNetSavingBytes - ...
    controlAttemptedBytes - candidatePayloadBytes;
feasibleMask = feasibleMask & (netSavingBytes >= -1e-9);

plan = struct();
plan.contractVersion = ...
    'fusion-aware-source-offer-exchange-plan-v229-v1';
plan.protocolId = protocol.id;
plan.sourceSensorIds = sourceSensorIds;
plan.sourceCount = sourceCount;
plan.maximumOffersPerSource = maximumOffersPerSource;
plan.solicitationHeaderBytes = offer.solicitationHeaderBytes;
plan.responseHeaderBytes = offer.responseHeaderBytes;
plan.offerRecordBytes = offer.offerRecordBytes;
plan.solicitationAttemptedBytes = solicitationAttemptedBytes;
plan.responseBytesPerSource = responseBytesPerSource;
plan.responseAttemptedBytes = responseAttemptedBytes;
plan.controlAttemptedBytes = controlAttemptedBytes;
plan.controlAuthorized = controlDecision.lightSynopsisAuthorized;
plan.controlDecision = controlDecision;
plan.spendableCreditAfterControlBytes = ...
    creditAfterOffers.spendableCreditBytes;
plan.protectedSavingAfterControlBytes = ...
    creditAfterOffers.protectedSavingBytes;
plan.candidatePayloadBytes = candidatePayloadBytes;
plan.candidateFeasibleMask = feasibleMask;
plan.feasibleCandidateCount = nnz(feasibleMask);
plan.certifiedNetSavingBytes = netSavingBytes;
plan.certifiedNetSavingPercent = 100 * netSavingBytes / ...
    max(referencePageBytes, eps);
plan.sourceRankingImplemented = offer.sourceRankingImplemented;
plan.sourceRankingValidated = offer.sourceRankingValidated;
plan.truthUsed = false;
plan.futureInformationUsed = false;
plan.modelMayOverrideByteFeasibility = false;
plan.evidenceBoundary = protocol.evidenceBoundary;
end

function valid = validNonnegativeScalar(value)
valid = isscalar(value) && isnumeric(value) && ...
    isfinite(value) && value >= 0;
end

function valid = validPositiveScalar(value)
valid = validNonnegativeScalar(value) && value > 0;
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
