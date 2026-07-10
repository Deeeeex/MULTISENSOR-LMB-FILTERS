function [objects, metadata] = decodeLmbWireMessage(bytes, model)
% DECODELMBWIREMESSAGE Decode and validate a v1 LMB application message.

schema = getLmbWireSchema();
stateDimension = validateModel(model, schema);
if ~isa(bytes, 'uint8') || ~isvector(bytes)
    error('decodeLmbWireMessage:InvalidBytes', ...
        'Message must be a uint8 vector.');
end
bytes = reshape(bytes, 1, []);
if numel(bytes) < schema.headerBytes
    error('decodeLmbWireMessage:TruncatedHeader', ...
        'Message is shorter than the fixed header.');
end
if ~isequal(bytes(1:4), schema.magic)
    error('decodeLmbWireMessage:BadMagic', 'Message magic is not LMBW.');
end
version = bytes(5);
eventType = bytes(6);
encodedDimension = bytes(7);
flags = bytes(8);
if version ~= schema.version
    error('decodeLmbWireMessage:UnsupportedVersion', ...
        'Unsupported message version.');
end
if eventType ~= schema.lightEventType && eventType ~= schema.fullEventType
    error('decodeLmbWireMessage:InvalidEventType', ...
        'Unsupported event type.');
end
if double(encodedDimension) ~= stateDimension
    error('decodeLmbWireMessage:StateDimensionMismatch', ...
        'Encoded state dimension does not match the model.');
end
if flags ~= schema.flags
    error('decodeLmbWireMessage:UnsupportedFlags', ...
        'Message flags do not describe the v1 packed-binary64 format.');
end

sender = unpackLittleEndian(bytes(9:10), 'uint16');
receiver = unpackLittleEndian(bytes(11:12), 'uint16');
timeIndex = unpackLittleEndian(bytes(13:16), 'uint32');
objectCountRaw = unpackLittleEndian(bytes(17:20), 'uint32');
totalBytesRaw = unpackLittleEndian(bytes(21:24), 'uint32');
objectCount = double(objectCountRaw);
totalBytes = double(totalBytesRaw);
if totalBytes ~= numel(bytes)
    error('decodeLmbWireMessage:TotalLengthMismatch', ...
        'Header totalBytes does not equal the byte vector length.');
end
remainingBytes = totalBytes - schema.headerBytes;
minimumComponentBytes = schema.floatBytes * (1 + stateDimension + ...
    stateDimension * (stateDimension + 1) / 2);
minimumObjectBytes = schema.objectHeaderBytes + minimumComponentBytes;
if objectCount > floor(remainingBytes / minimumObjectBytes)
    error('decodeLmbWireMessage:ObjectCountExceedsMessage', ...
        ['Object count and minimum component payload cannot fit in the ' ...
         'remaining message bytes.']);
end

metadata = struct( ...
    'version', version, ...
    'eventType', eventType, ...
    'stateDimension', encodedDimension, ...
    'flags', flags, ...
    'sender', sender, ...
    'receiver', receiver, ...
    'timeIndex', timeIndex, ...
    'objectCount', objectCountRaw, ...
    'totalBytes', totalBytesRaw);
if objectCount == 0
    objects = model.object([]);
    if remainingBytes ~= 0
        error('decodeLmbWireMessage:TrailingBytes', ...
            'Empty message contains trailing payload bytes.');
    end
    return;
end

template = instantiateTemplate(model.object, stateDimension);
objects = repmat(template, 1, objectCount);
componentBytes = schema.floatBytes * (1 + stateDimension + ...
    stateDimension * (stateDimension + 1) / 2);
upperMask = triu(true(stateDimension));
upperValueCount = nnz(upperMask);
cursor = schema.headerBytes + 1;
for objectIdx = 1:objectCount
    requireRemaining(cursor, schema.objectHeaderBytes, totalBytes, ...
        'decodeLmbWireMessage:TruncatedObjectHeader');
    birthTime = unpackLittleEndian(bytes(cursor:(cursor + 3)), 'uint32');
    cursor = cursor + 4;
    birthLocation = ...
        unpackLittleEndian(bytes(cursor:(cursor + 3)), 'uint32');
    cursor = cursor + 4;
    existence = unpackLittleEndian(bytes(cursor:(cursor + 7)), 'double');
    cursor = cursor + 8;
    componentCountRaw = ...
        unpackLittleEndian(bytes(cursor:(cursor + 1)), 'uint16');
    cursor = cursor + 2;
    reserved = unpackLittleEndian(bytes(cursor:(cursor + 1)), 'uint16');
    cursor = cursor + 2;
    componentCount = double(componentCountRaw);
    if ~isfinite(existence) || existence < 0 || existence > 1
        error('decodeLmbWireMessage:InvalidExistence', ...
            'Decoded existence must be finite and in [0,1].');
    end
    if reserved ~= 0
        error('decodeLmbWireMessage:ReservedFieldNonzero', ...
            'Reserved object-header bits must be zero.');
    end
    if componentCount < 1
        error('decodeLmbWireMessage:InvalidComponentCount', ...
            'Each encoded object requires at least one component.');
    end
    if eventType == schema.lightEventType && componentCount ~= 1
        error('decodeLmbWireMessage:LightComponentCount', ...
            'Light messages require one component per object.');
    end
    bytesNeeded = componentCount * componentBytes;
    requireRemaining(cursor, bytesNeeded, totalBytes, ...
        'decodeLmbWireMessage:TruncatedComponents');

    weights = zeros(1, componentCount);
    means = cell(1, componentCount);
    covariances = cell(1, componentCount);
    for componentIdx = 1:componentCount
        weights(componentIdx) = ...
            unpackLittleEndian(bytes(cursor:(cursor + 7)), 'double');
        cursor = cursor + 8;
        means{componentIdx} = reshape(unpackLittleEndian( ...
            bytes(cursor:(cursor + 8 * stateDimension - 1)), ...
            'double'), stateDimension, 1);
        cursor = cursor + 8 * stateDimension;
        upperValues = unpackLittleEndian(bytes( ...
            cursor:(cursor + 8 * upperValueCount - 1)), 'double');
        cursor = cursor + 8 * upperValueCount;
        covariance = zeros(stateDimension);
        covariance(upperMask) = upperValues;
        covariance = covariance + triu(covariance, 1)';
        covariances{componentIdx} = covariance;
    end
    if any(~isfinite(weights)) || any(weights < 0) || sum(weights) <= 0
        error('decodeLmbWireMessage:InvalidWeights', ...
            'Decoded weights must be finite, nonnegative, and have positive sum.');
    end
    for componentIdx = 1:componentCount
        if any(~isfinite(means{componentIdx})) || ...
                any(~isfinite(covariances{componentIdx}(:)))
            error('decodeLmbWireMessage:NonfiniteComponent', ...
                'Decoded component values must be finite.');
        end
    end

    objects(objectIdx).birthTime = double(birthTime);
    objects(objectIdx).birthLocation = double(birthLocation);
    objects(objectIdx).r = double(existence);
    objects(objectIdx).numberOfGmComponents = componentCount;
    objects(objectIdx).w = weights;
    objects(objectIdx).mu = means;
    objects(objectIdx).Sigma = covariances;
    if isfield(objects, 'trajectoryLength')
        objects(objectIdx).trajectoryLength = 0;
    end
    if isfield(objects, 'trajectory')
        objects(objectIdx).trajectory = zeros(stateDimension, 0);
    end
    if isfield(objects, 'timestamps')
        objects(objectIdx).timestamps = zeros(1, 0);
    end
end
if cursor ~= totalBytes + 1
    error('decodeLmbWireMessage:TrailingBytes', ...
        'Message contains bytes not described by its object counts.');
end
end

function stateDimension = validateModel(model, schema)
if ~isstruct(model) || ~isfield(model, 'xDimension') || ...
        ~isfield(model, 'object') || ~isstruct(model.object) || ...
        numel(model.object) > 1 || ...
        ~(isnumeric(model.xDimension) || islogical(model.xDimension)) || ...
        ~isreal(model.xDimension) || ~isscalar(model.xDimension) || ...
        ~isfinite(model.xDimension) || ...
        model.xDimension ~= floor(model.xDimension) || ...
        model.xDimension < 1 || ...
        model.xDimension > schema.maxStateDimension
    error('decodeLmbWireMessage:InvalidModel', ...
        'model must provide a valid state dimension and object template.');
end
stateDimension = double(model.xDimension);
end

function template = instantiateTemplate(emptyTemplate, stateDimension)
required = {'birthLocation', 'birthTime', 'r', ...
    'numberOfGmComponents', 'w', 'mu', 'Sigma'};
for fieldIdx = 1:numel(required)
    if ~isfield(emptyTemplate, required{fieldIdx})
        error('decodeLmbWireMessage:InvalidTemplate', ...
            'model.object is missing field %s.', required{fieldIdx});
    end
end
if isempty(emptyTemplate)
    template = emptyTemplate;
    template(1).birthLocation = 0;
else
    template = emptyTemplate(1);
end
template.birthLocation = 0;
template.birthTime = 0;
template.r = 0;
template.numberOfGmComponents = 0;
template.w = zeros(1, 0);
template.mu = cell(1, 0);
template.Sigma = cell(1, 0);
if isfield(template, 'trajectoryLength')
    template.trajectoryLength = 0;
end
if isfield(template, 'trajectory')
    template.trajectory = zeros(stateDimension, 0);
end
if isfield(template, 'timestamps')
    template.timestamps = zeros(1, 0);
end
end

function requireRemaining(cursor, byteCount, totalBytes, errorId)
if byteCount < 0 || cursor < 1 || ...
        byteCount > totalBytes - cursor + 1
    error(errorId, 'Message ended before the declared payload was complete.');
end
end

function value = unpackLittleEndian(bytes, className)
bytes = reshape(uint8(bytes), 1, []);
value = typecast(bytes, className);
if ~hostIsLittleEndian()
    value = swapbytes(value);
end
value = reshape(value, 1, []);
if isscalar(value)
    value = value(1);
end
end

function value = hostIsLittleEndian()
persistent cachedValue;
if isempty(cachedValue)
    probe = typecast(uint16(1), 'uint8');
    cachedValue = probe(1) == 1;
end
value = cachedValue;
end
