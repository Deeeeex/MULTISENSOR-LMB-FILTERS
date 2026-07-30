function [reportPath, summary] = ...
    runLabelSetAdaptiveDominantPayloadAwareRefinementProbeM24( ...
        payloadMarginToken, options)
% RUNLABELSETADAPTIVEDOMINANTPAYLOADAWAREREFINEMENTPROBEM24 Seed-31 probe.

if nargin < 1 || isempty(payloadMarginToken)
    payloadMarginToken = 10;
end
if nargin < 2 || isempty(options)
    options = struct();
end
protocol = getLabelSetSimulatorPolicyProtocol();
if ~protocol.adaptiveDominantPayloadAwareRefinementAuthorized || ...
        ~ismember(payloadMarginToken, protocol. ...
            adaptiveDominantPayloadAwareRefinementMarginTokens)
    error('Payload-aware refinement token is not registered.');
end
runOptions = options;
runOptions.evidenceSplit = 'development';
[reportPath, summary] = ...
    runLabelSetAdaptiveDominantPayloadAwareM24( ...
        protocol.adaptiveDominantPayloadAwareRefinementProbeSeed, ...
        payloadMarginToken, runOptions);
end
