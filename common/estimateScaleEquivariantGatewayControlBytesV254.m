function details = ...
        estimateScaleEquivariantGatewayControlBytesV254( ...
            context, referenceAssignment, synopsisMode)
% ESTIMATESCALEEQUIVARIANTGATEWAYCONTROLBYTESV254 Charge policy telemetry.

protocol = getScaleEquivariantSafeGatewayV254Protocol();
if ~ischar(synopsisMode) || ...
        ~ismember(synopsisMode, protocol.controlSynopsisModes) || ...
        ~isstruct(context) || ~isscalar(context) || ...
        ~isfield(context, 'localPosteriorBySensor') || ...
        ~isnumeric(referenceAssignment) || ...
        isempty(referenceAssignment) || size(referenceAssignment, 2) ~= 4
    error('ScaleEquivariantGatewayV254:InvalidControlCostInput', ...
        'The V254 control-byte request is malformed.');
end

nodeCount = numel(context.localPosteriorBySensor);
arcCount = size(referenceAssignment, 1);
activeLabelCounts = zeros(1, nodeCount);
switch synopsisMode
    case 'compact-node-32'
        synopsisBytes = protocol.compactNodeBytesPerSensor * nodeCount;
    case 'rich-active-label-64'
        for sensorIdx = 1:nodeCount
            objects = context.localPosteriorBySensor{sensorIdx};
            if ~isempty(objects)
                activeLabelCounts(sensorIdx) = sum( ...
                    [objects.r] >= ...
                    protocol.activeLabelExistenceThreshold);
            end
        end
        synopsisBytes = sum(protocol.richHeaderBytesPerSensor + ...
            protocol.richBytesPerActiveLabel * activeLabelCounts);
end
routeCommandBytes = protocol.routeCommandHeaderBytes + ...
    protocol.routeCommandBytesPerDirectedGateway * arcCount;

details = struct();
details.contractVersion = ...
    'scale-equivariant-gateway-v254-control-byte-estimate-v1';
details.synopsisMode = synopsisMode;
details.nodeCount = nodeCount;
details.directedGatewayCount = arcCount;
details.activeLabelCounts = activeLabelCounts;
details.synopsisAttemptedBytes = synopsisBytes;
details.routeCommandAttemptedBytes = routeCommandBytes;
details.totalAttemptedBytes = synopsisBytes + routeCommandBytes;
details.controlTrafficIncludedInPosteriorLedger = false;
details.truthUsed = false;
details.futureInformationUsed = false;
end
