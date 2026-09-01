function bank = buildEtaSafeDominantEdgeReplacementV225ActionBank( ...
        context, groupIds, options)
% BUILDETASAFEDOMINANTEDGEREPLACEMENTV225ACTIONBANK Frozen V225 arm.

if nargin < 3 || isempty(options)
    options = struct();
end
protocol = getEtaSafeDominantEdgeReplacementV225Protocol();
options.inputRouteWeightMode = protocol.inputRouteWeightMode;
options.inputRouteEtaProjectionEnabled = true;
options.inputRouteMinimumMapExistence = getField(options, ...
    'inputRouteMinimumMapExistence', protocol.minimumMapExistence);
options.inputRouteMaximumReferenceLogOddsDrop = getField(options, ...
    'inputRouteMaximumReferenceLogOddsDrop', ...
    protocol.maximumReferenceLogOddsDrop);
options.inputRouteEtaIdentityTolerance = getField(options, ...
    'inputRouteEtaIdentityTolerance', protocol.identityTolerance);
bank = buildEtaSafeSemanticInputRoutingV223ActionBank( ...
    context, groupIds, options);

actionIdx = find((1:bank.actionCount) ~= bank.referenceActionIndex);
if numel(actionIdx) ~= 1 || ...
        ~strcmp(bank.inputRouteWeightMode, ...
            protocol.inputRouteWeightMode)
    error('EtaSafeDominantEdgeReplacementV225:CarrierDrift', ...
        'V225 requires exactly one eta-safe routed-label action.');
end
bank.actionNames{actionIdx} = sprintf( ...
    ['v225-eta-safe-donor-f%d-to-beneficiary-f%d-', ...
     'replace-dominant-nonself-s%d-l%d-%d'], ...
    bank.donorFormationId, bank.beneficiaryFormationId, ...
    bank.semanticShortcutSourceId, ...
    bank.semanticShortcutLabel(1), bank.semanticShortcutLabel(2));
bank.contractVersion = ...
    'eta-safe-dominant-edge-replacement-action-bank-v225-v1';
bank.bankVariant = 'eta-safe-dominant-edge-replacement-v225';
bank.protocolId = protocol.id;
bank.semanticFusionMode = protocol.semanticFusionMode;
bank.dominantNonselfTransferEnabled = true;
bank.selfInputReplacementAllowed = false;
bank.totalLabelWeightPreserved = true;
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
