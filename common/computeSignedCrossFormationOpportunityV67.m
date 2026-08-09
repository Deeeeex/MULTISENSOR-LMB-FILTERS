function metrics = computeSignedCrossFormationOpportunityV67( ...
        baseMetrics, minimumMaterialPressureFraction)
% COMPUTESIGNEDCROSSFORMATIONOPPORTUNITYV67 Separate action signs.
%
% Withholding one formation's incoming cross-formation bundle can have two
% opposite observable consequences. Receiver-supported existence restored
% by withholding is quarantine pressure; sender-supported existence lost by
% withholding is transport-retention pressure. Both use the same V65 network
% denominator, so their difference has a meaningful sign across scales.

if nargin < 2 || isempty(minimumMaterialPressureFraction)
    minimumMaterialPressureFraction = 0.01;
end
required = { ...
    'groups', 'singleActionAvailableMask', ...
    'networkRescueFractionByFormation', ...
    'networkUsefulLossFractionByFormation', ...
    'commonNetworkDenominatorUsed'};
if ~isstruct(baseMetrics) || ~isscalar(baseMetrics) || ...
        ~all(isfield(baseMetrics, required)) || ...
        ~isequal(baseMetrics.commonNetworkDenominatorUsed, true) || ...
        ~isscalar(minimumMaterialPressureFraction) || ...
        ~isfinite(minimumMaterialPressureFraction) || ...
        minimumMaterialPressureFraction < 0
    error('SignedOpportunityV67:InvalidInput', ...
        'V67 requires one network-normalized V65 metric record.');
end

groups = reshape(baseMetrics.groups, 1, []);
available = reshape(baseMetrics.singleActionAvailableMask, 1, []);
quarantine = reshape( ...
    baseMetrics.networkRescueFractionByFormation, 1, []);
transport = reshape( ...
    baseMetrics.networkUsefulLossFractionByFormation, 1, []);
if numel(available) ~= numel(groups) || ...
        numel(quarantine) ~= numel(groups) || ...
        numel(transport) ~= numel(groups)
    error('SignedOpportunityV67:FormationMismatch', ...
        'The signed formation pressures do not align.');
end
valid = available & isfinite(quarantine) & isfinite(transport);
quarantine(~valid) = NaN;
transport(~valid) = NaN;
totalQuarantine = sum(quarantine(valid));
totalTransport = sum(transport(valid));
totalMagnitude = totalQuarantine + totalTransport;
signedBalance = totalTransport - totalQuarantine;
transportShare = 0;
if totalMagnitude > eps
    transportShare = totalTransport / totalMagnitude;
end

materialQuarantine = ...
    totalQuarantine >= minimumMaterialPressureFraction - 1e-12;
materialTransport = ...
    totalTransport >= minimumMaterialPressureFraction - 1e-12;
if materialTransport && materialQuarantine
    regime = 'mixed';
elseif materialTransport
    regime = 'transport';
elseif materialQuarantine
    regime = 'quarantine';
else
    regime = 'neutral';
end

metrics = struct();
metrics.contractVersion = ...
    'signed-cross-formation-opportunity-v67-v1';
metrics.groups = groups;
metrics.availableFormationMask = valid;
metrics.quarantinePressureByFormation = quarantine;
metrics.transportRetentionPressureByFormation = transport;
metrics.signedTransportBalanceByFormation = transport - quarantine;
metrics.totalQuarantinePressure = totalQuarantine;
metrics.totalTransportRetentionPressure = totalTransport;
metrics.totalSignedTransportBalance = signedBalance;
metrics.totalPressureMagnitude = totalMagnitude;
metrics.transportShare = transportShare;
metrics.minimumMaterialPressureFraction = ...
    minimumMaterialPressureFraction;
metrics.regime = regime;
metrics.truthUsed = false;
metrics.futureMeasurementsUsed = false;
metrics.futureOutcomesUsed = false;
end
