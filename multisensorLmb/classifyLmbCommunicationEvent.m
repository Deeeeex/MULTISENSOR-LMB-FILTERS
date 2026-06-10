function [eventType, details] = classifyLmbCommunicationEvent( ...
    utility, utilityDetails, config, referenceAge)
% CLASSIFYLMBCOMMUNICATIONEVENT Apply single or dual trigger thresholds.
%
% Event codes are 0=none, 1=light, and 2=heavy.

if nargin < 4 || isempty(referenceAge)
    referenceAge = inf;
end
policy = lower(getField(config, 'eventPolicy', 'dual'));
criterionMode = lower(getField(config, 'criterionMode', 'multi'));
thresholdLow = getField(config, 'thresholdLow', 0.25);
thresholdHigh = getField(config, 'thresholdHigh', 0.60);
if thresholdHigh < thresholdLow
    tmp = thresholdLow;
    thresholdLow = thresholdHigh;
    thresholdHigh = tmp;
end

if strcmp(criterionMode, 'informationonly')
    criterion = getField(utilityDetails, 'maxInformationGain', 0);
else
    criterion = utility;
end

forceHeavy = false;
if getField(config, 'forceInitialHeavy', true) && ...
        getField(utilityDetails, 'referenceEmpty', true) && ...
        ~getField(utilityDetails, 'currentEmpty', false)
    forceHeavy = true;
end
if getField(config, 'forceLabelChangeHeavy', true) && ...
        getField(utilityDetails, 'labelChanged', false)
    forceHeavy = true;
end
maxReferenceAge = getField(config, 'maxReferenceAge', 10);
if getField(config, 'forceStaleHeavy', true) && ...
        referenceAge > maxReferenceAge
    forceHeavy = true;
end

switch policy
    case 'none'
        eventType = 0;
        forceHeavy = false;
    case 'alwayslight'
        eventType = 1;
    case 'alwaysheavy'
        eventType = 2;
    case 'singleheavy'
        eventType = 2 * double(criterion >= thresholdLow);
    otherwise
        if criterion >= thresholdHigh
            eventType = 2;
        elseif criterion >= thresholdLow
            eventType = 1;
        else
            eventType = 0;
        end
end

if forceHeavy && ~strcmp(policy, 'none')
    eventType = 2;
end
details = struct('criterion', criterion, 'forceHeavy', forceHeavy, ...
    'thresholdLow', thresholdLow, 'thresholdHigh', thresholdHigh);
end

function value = getField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end
