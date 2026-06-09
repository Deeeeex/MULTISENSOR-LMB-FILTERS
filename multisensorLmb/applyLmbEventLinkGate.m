function [eventType, gateDetails] = applyLmbEventLinkGate( ...
    rawEventType, nominalReliability, recentSuccessRate, isOutage, config)
% APPLYLMBEVENTLINKGATE Downgrade events when the directed link is weak.

if nargin < 5 || isempty(config)
    config = struct();
end
if ~isfinite(recentSuccessRate)
    linkQuality = nominalReliability;
else
    linkQuality = 0.5 * nominalReliability + 0.5 * recentSuccessRate;
end
linkQuality = min(max(linkQuality, 0), 1);
eventType = rawEventType;
reason = 'unchanged';

if isOutage
    eventType = 0;
    reason = 'outage';
elseif ~getField(config, 'linkGateEnabled', true)
    reason = 'gate-disabled';
else
    poorThreshold = getField(config, 'poorLinkThreshold', 0.35);
    moderateThreshold = getField(config, 'moderateLinkThreshold', 0.70);
    if linkQuality < poorThreshold
        if rawEventType == 1
            eventType = 0;
            reason = 'poor-link-cancel-light';
        elseif rawEventType == 2
            eventType = 1;
            reason = 'poor-link-downgrade-heavy';
        end
    elseif linkQuality < moderateThreshold && rawEventType == 2
        eventType = 1;
        reason = 'moderate-link-downgrade-heavy';
    end
end

gateDetails = struct('linkQuality', linkQuality, ...
    'wasDowngraded', eventType < rawEventType, 'reason', reason);
end

function value = getField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end
