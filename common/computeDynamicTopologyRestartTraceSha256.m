function trace = computeDynamicTopologyRestartTraceSha256( ...
    stateEstimatesBySensor, diagnostics, times)
% COMPUTEDYNAMICTOPOLOGYRESTARTTRACESHA256 Deterministic restart trace.
%
% The trace hashes post-fusion estimates, selected topology, fusion
% weights, communication accounting and local-update diagnostics. It is
% intended to prove that restarting from a saved predecision state
% reproduces the corresponding segment of the uninterrupted behavior run.

times = reshape(times, 1, []);
if isempty(times) || any(~isfinite(times)) || ...
        any(times ~= round(times)) || any(times < 1)
    error('Restart-trace times must be positive integer timestamps.');
end
estimateText = serializeEstimates( ...
    stateEstimatesBySensor, times);
selectedText = serializeNumeric( ...
    diagnostics.topologyActiveEdge(:, :, times));
weightText = serializeCellSlice( ...
    diagnostics.topologyPolicyFusionWeightMatrix, times);
attemptedText = serializeNumeric( ...
    diagnostics.attemptedPayloadBytes(:, :, times));
deliveredText = serializeNumeric( ...
    diagnostics.payloadBytes(:, :, times));
updateText = [ ...
    serializeNumeric(diagnostics.localInnovation(:, times)), ...
    serializeNumeric( ...
        diagnostics.localAssociationConfidence(:, times)), ...
    serializeNumeric(diagnostics.localNisNorm(:, times)), ...
    serializeNumeric(diagnostics.localNisDeviation(:, times))];
messageText = serializeNumeric( ...
    diagnostics.topologyDirectedMessageCount(times));

trace = struct();
trace.contractVersion = ...
    'dynamic-topology-restart-trace-sha256-v1';
trace.times = times;
trace.estimateSha256 = computeTextSha256(estimateText);
trace.selectedTopologySha256 = ...
    computeTextSha256(selectedText);
trace.fusionWeightSha256 = ...
    computeTextSha256(weightText);
trace.attemptedPayloadSha256 = ...
    computeTextSha256(attemptedText);
trace.deliveredPayloadSha256 = ...
    computeTextSha256(deliveredText);
trace.localUpdateSha256 = ...
    computeTextSha256(updateText);
trace.messageCountSha256 = ...
    computeTextSha256(messageText);
trace.overallSha256 = computeTextSha256([ ...
    trace.contractVersion, '|', ...
    serializeNumeric(times), ...
    trace.estimateSha256, ...
    trace.selectedTopologySha256, ...
    trace.fusionWeightSha256, ...
    trace.attemptedPayloadSha256, ...
    trace.deliveredPayloadSha256, ...
    trace.localUpdateSha256, ...
    trace.messageCountSha256]);
end

function text = serializeEstimates(estimates, times)
parts = cell(1, numel(estimates) * numel(times) * 3);
partIdx = 0;
for sensorIdx = 1:numel(estimates)
    estimate = estimates{sensorIdx};
    required = {'labels', 'mu', 'Sigma'};
    for fieldIdx = 1:numel(required)
        if ~isfield(estimate, required{fieldIdx})
            error('Restart-trace estimate lacks a required field.');
        end
    end
    for timeIdx = 1:numel(times)
        currentTime = times(timeIdx);
        partIdx = partIdx + 1;
        parts{partIdx} = serializeNumeric( ...
            estimate.labels{currentTime});
        partIdx = partIdx + 1;
        parts{partIdx} = serializeNumericCell( ...
            estimate.mu{currentTime});
        partIdx = partIdx + 1;
        parts{partIdx} = serializeNumericCell( ...
            estimate.Sigma{currentTime});
    end
end
text = [parts{1:partIdx}];
end

function text = serializeCellSlice(values, indices)
parts = cell(1, numel(indices));
for idx = 1:numel(indices)
    parts{idx} = serializeNumeric(values{indices(idx)});
end
text = [parts{:}];
end

function text = serializeNumericCell(values)
if ~iscell(values)
    error('Restart-trace estimate mixtures must be cell arrays.');
end
parts = cell(1, numel(values) + 1);
parts{1} = sprintf('cell:%d|', numel(values));
for idx = 1:numel(values)
    parts{idx + 1} = serializeNumeric(values{idx});
end
text = [parts{:}];
end

function text = serializeNumeric(value)
if ~(isnumeric(value) || islogical(value))
    error('Restart-trace values must be numeric or logical.');
end
sizeText = sprintf('%d,', size(value));
if islogical(value)
    valueText = sprintf('%d,', value(:));
else
    valueText = sprintf('%.17g,', value(:));
end
text = sprintf('%s:%s|%s|', ...
    class(value), sizeText, valueText);
end
