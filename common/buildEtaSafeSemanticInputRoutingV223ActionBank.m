function bank = buildEtaSafeSemanticInputRoutingV223ActionBank( ...
        context, groupIds, options)
% BUILDETASAFESEMANTICINPUTROUTINGV223ACTIONBANK Frozen V223 arm.

if nargin < 3 || isempty(options)
    options = struct();
end
protocol = getEtaSafeSemanticInputRoutingV223Protocol();
options.inputRouteEtaProjectionEnabled = true;
options.inputRouteMinimumMapExistence = getField(options, ...
    'inputRouteMinimumMapExistence', protocol.minimumMapExistence);
options.inputRouteMaximumReferenceLogOddsDrop = getField(options, ...
    'inputRouteMaximumReferenceLogOddsDrop', ...
    protocol.maximumReferenceLogOddsDrop);
options.inputRouteEtaIdentityTolerance = getField(options, ...
    'inputRouteEtaIdentityTolerance', protocol.identityTolerance);
bank = buildSinglePassSemanticInputRoutingV221ActionBank( ...
    context, groupIds, options);

actionIdx = find((1:bank.actionCount) ~= bank.referenceActionIndex);
if numel(actionIdx) ~= 1
    error('EtaSafeSemanticInputRoutingV223:CarrierDrift', ...
        'V223 requires exactly one routed-label action.');
end
bank.actionNames{actionIdx} = strrep( ...
    bank.actionNames{actionIdx}, 'v221-', 'v223-eta-safe-');
bank.contractVersion = ...
    'eta-safe-semantic-input-routing-action-bank-v223-v1';
bank.bankVariant = 'eta-safe-semantic-input-routing-v223';
bank.protocolId = protocol.id;
bank.etaProjectionEnabled = true;
bank.minimumMapExistence = ...
    options.inputRouteMinimumMapExistence;
bank.maximumReferenceLogOddsDrop = ...
    options.inputRouteMaximumReferenceLogOddsDrop;
bank.etaIdentityTolerance = ...
    options.inputRouteEtaIdentityTolerance;
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
