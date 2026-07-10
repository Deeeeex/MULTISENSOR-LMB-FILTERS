function stats = estimateLmbPayloadSize(objects, model, eventType, diagnostics)
% ESTIMATELMBPAYLOADSIZE Legacy scalar-equivalent LMB field counter.
%
% eventType: 0=none, 1=light moment-matched LMB, 2=heavy full GM-LMB.
% This helper predates the typed application-layer wire codec. Its
% estimatedBytes field assumes every counted value occupies one binary64
% scalar and is retained only for historical tests and reports. Production
% communication and paper-facing byte metrics must use numel(bytes) from
% encodeLmbWireMessage instead.

if nargin < 4
    diagnostics = struct();
end
stats = struct('scalarCount', 0, 'estimatedBytes', 0, ...
    'objectCount', 0, 'componentCount', 0);
if eventType <= 0 || isempty(objects)
    return;
end

stateDimension = model.xDimension;
activeThreshold = getField(model, 'existenceThreshold', 0);
active = [objects.r] > activeThreshold & ...
    [objects.numberOfGmComponents] > 0;
objects = objects(active);
stats.objectCount = numel(objects);

% Message header: sender, receiver, time, event type, plus compact diagnostics.
headerScalars = 4 + countDiagnosticScalars(diagnostics);
payloadScalars = 0;
for idx = 1:numel(objects)
    componentCount = objects(idx).numberOfGmComponents;
    if eventType == 1
        componentCount = min(componentCount, 1);
    end
    stats.componentCount = stats.componentCount + componentCount;
    % label pair + existence probability
    payloadScalars = payloadScalars + 3;
    % mixture weight + mean + full covariance per Gaussian component
    payloadScalars = payloadScalars + componentCount * ...
        (1 + stateDimension + stateDimension * stateDimension);
end

stats.scalarCount = headerScalars + payloadScalars;
stats.estimatedBytes = 8 * stats.scalarCount;
end

function count = countDiagnosticScalars(diagnostics)
count = 0;
fields = {'associationConfidence', 'innovationScore', ...
    'innovationNovelty', 'nisAgg', 'nisNorm', 'nisDeviation'};
for idx = 1:numel(fields)
    if isstruct(diagnostics) && isfield(diagnostics, fields{idx})
        count = count + numel(diagnostics.(fields{idx}));
    end
end
end

function value = getField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end
