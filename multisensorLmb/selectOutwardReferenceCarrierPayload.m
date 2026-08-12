function [payload, applied, pageIdx] = ...
        selectOutwardReferenceCarrierPayload( ...
            currentPayload, config, senderIdx, receiverIdx, ...
            currentTime, eventType, isReceiverSafeEdge)
% SELECTOUTWARDREFERENCECARRIERPAYLOAD Apply one frozen edge-time override.

payload = currentPayload;
applied = false;
pageIdx = 0;
if ~config.outwardReferenceCarrierEnabled
    return;
end
pageIdx = find(config.outwardReferenceCarrierTimes == currentTime, 1);
if isempty(pageIdx) || ...
        config.outwardReferenceCarrierSenderIds(pageIdx) ~= senderIdx || ...
        config.outwardReferenceCarrierReceiverIds(pageIdx) ~= receiverIdx
    pageIdx = 0;
    return;
end
if eventType ~= 2 || isReceiverSafeEdge
    error('OutwardReferenceCarrierV125:InvalidMessageOpportunity', ...
        ['Reference-carrier substitution requires a full-payload ', ...
         'non-selective message opportunity.']);
end
objects = config.outwardReferenceCarrierPosteriorByTime{pageIdx};
if isempty(objects)
    payload = objects;
else
    active = [objects.r] > config.payloadExistenceThreshold & ...
        [objects.numberOfGmComponents] > 0;
    payload = objects(active);
end
applied = true;
end
