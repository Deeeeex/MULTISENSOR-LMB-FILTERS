function report = preflightOutwardReferenceCarrierV125X36T72()
% PREFLIGHTOUTWARDREFERENCECARRIERV125X36T72 Real X36 structural gate.

protocol = getOutwardReferenceCarrierV125Protocol();
report = preflightProjectedFormationRowCompositionV123X36T72(protocol);
report.carrierTimes = protocol.carrierTimes;
report.carrierSenderSensorId = protocol.carrierSenderSensorId;
report.carrierReceiverSensorId = protocol.carrierReceiverSensorId;
end
