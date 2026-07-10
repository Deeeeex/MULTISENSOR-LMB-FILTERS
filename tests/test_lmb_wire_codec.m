function test_lmb_wire_codec()
% TEST_LMB_WIRE_CODEC Verify the typed LMB application-layer wire format.

model = generateMultisensorModel( ...
    2, [1, 1], [0.9, 0.9], [3, 3], 'GA', 'LBP');
stateDimension = model.xDimension;
schema = getLmbWireSchema();
assert(schema.headerBytes == 24);
assert(schema.objectHeaderBytes == 20);
assert(schema.version == 1);
assert(schema.flags == 3);

emptyMetadata = makeMetadata(1, hex2dec('0102'), ...
    hex2dec('0304'), hex2dec('01020304'));
[emptyBytes, emptyStats] = ...
    encodeLmbWireMessage(model.object, emptyMetadata, model);
expectedHeader = uint8([ ...
    hex2dec('4c'), hex2dec('4d'), hex2dec('42'), hex2dec('57'), ...
    1, 1, stateDimension, 3, ...
    hex2dec('02'), hex2dec('01'), ...
    hex2dec('04'), hex2dec('03'), ...
    hex2dec('04'), hex2dec('03'), hex2dec('02'), hex2dec('01'), ...
    0, 0, 0, 0, 24, 0, 0, 0]);
assert(isa(emptyBytes, 'uint8'));
assert(isrow(emptyBytes));
assert(isequal(emptyBytes, expectedHeader));
assert(emptyStats.encodedBytes == schema.headerBytes);
[decodedEmpty, decodedEmptyMetadata] = ...
    decodeLmbWireMessage(emptyBytes, model);
assert(isempty(decodedEmpty));
assert(isequal(fieldnames(decodedEmpty), fieldnames(model.object)));
assertMetadata(decodedEmptyMetadata, emptyMetadata, 0, schema.headerBytes);

light = makeObject(model, 7, 11, 1, 1);
light.w = 1;
light.mu = {zeros(stateDimension, 1)};
upperMask = triu(true(stateDimension));
knownUpper = (1:nnz(upperMask))';
knownCovariance = zeros(stateDimension);
knownCovariance(upperMask) = knownUpper;
knownCovariance = knownCovariance + triu(knownCovariance, 1)';
light.Sigma = {knownCovariance};
lightMetadata = makeMetadata(1, 1, 2, 9);
[lightBytes, lightStats] = ...
    encodeLmbWireMessage(light, lightMetadata, model);
componentBytes = 8 * (1 + stateDimension + ...
    stateDimension * (stateDimension + 1) / 2);
expectedLightBytes = 24 + 20 + componentBytes;
assert(numel(lightBytes) == expectedLightBytes);
assert(lightStats.encodedBytes == expectedLightBytes);
assert(isequal(lightBytes(33:40), ...
    uint8([0, 0, 0, 0, 0, 0, 240, 63]))); % r = 1.0
assert(isequal(lightBytes(45:52), ...
    uint8([0, 0, 0, 0, 0, 0, 240, 63]))); % w = 1.0
assert(isequal(lightBytes(85:92), ...
    uint8([0, 0, 0, 0, 0, 0, 240, 63]))); % Sigma upper #1
assert(isequal(lightBytes(93:100), ...
    uint8([0, 0, 0, 0, 0, 0, 0, 64])));   % Sigma upper #2
assert(isequal(lightBytes(101:108), ...
    uint8([0, 0, 0, 0, 0, 0, 8, 64])));   % Sigma upper #3
[decodedLight, decodedLightMetadata] = ...
    decodeLmbWireMessage(lightBytes, model);
assertTransportFieldsEqual(decodedLight, light);
assertTemplateFields(decodedLight, model);
assertMetadata(decodedLightMetadata, lightMetadata, 1, expectedLightBytes);

full = makeObject(model, 8, 12, 0.8, 2);
fullMetadata = makeMetadata(2, 2, 1, 10);
[fullBytes, fullStats] = encodeLmbWireMessage(full, fullMetadata, model);
expectedFullBytes = 24 + 20 + 2 * componentBytes;
assert(numel(fullBytes) == expectedFullBytes);
assert(expectedLightBytes < expectedFullBytes);
assert(fullStats.componentCount == 2);
[decodedFull, decodedFullMetadata] = ...
    decodeLmbWireMessage(fullBytes, model);
assertTransportFieldsEqual(decodedFull, full);
assertTemplateFields(decodedFull, model);
assertMetadata(decodedFullMetadata, fullMetadata, 1, expectedFullBytes);

mixed = [makeObject(model, 9, 1, 0.7, 1), ...
    makeObject(model, 9, 2, 0.6, 2)];
mixedMetadata = makeMetadata(2, 1, 2, 11);
[mixedBytes, mixedStats] = ...
    encodeLmbWireMessage(mixed, mixedMetadata, model);
expectedMixedBytes = 24 + 2 * 20 + 3 * componentBytes;
assert(numel(mixedBytes) == expectedMixedBytes);
assert(mixedStats.objectCount == 2);
assert(mixedStats.componentCount == 3);
[decodedMixed, decodedMixedMetadata] = ...
    decodeLmbWireMessage(mixedBytes, model);
assertTransportFieldsEqual(decodedMixed, mixed);
assertTemplateFields(decodedMixed, model);
assertMetadata(decodedMixedMetadata, mixedMetadata, 2, expectedMixedBytes);

approximatelySymmetric = makeObject(model, 10, 1, 0.75, 1);
approximatelySymmetric.Sigma{1}(1, 2) = 1;
approximatelySymmetric.Sigma{1}(2, 1) = 1 + eps(1);
canonicalCovariance = (approximatelySymmetric.Sigma{1} + ...
    approximatelySymmetric.Sigma{1}') / 2;
approximatelySymmetricBytes = encodeLmbWireMessage( ...
    approximatelySymmetric, lightMetadata, model);
decodedApproximatelySymmetric = decodeLmbWireMessage( ...
    approximatelySymmetricBytes, model);
assert(isequaln(decodedApproximatelySymmetric.Sigma{1}, ...
    canonicalCovariance));

testMalformedDecoderInputs( ...
    emptyBytes, lightBytes, fullBytes, model);
testEncoderValidation(light, full, lightMetadata, model);
fprintf('test_lmb_wire_codec passed\n');
end

function metadata = makeMetadata(eventType, sender, receiver, timeIndex)
metadata = struct( ...
    'eventType', eventType, ...
    'sender', sender, ...
    'receiver', receiver, ...
    'timeIndex', timeIndex);
end

function object = makeObject( ...
    model, birthTime, birthLocation, existence, componentCount)
object = model.object;
object(1).birthTime = birthTime;
object(1).birthLocation = birthLocation;
object(1).r = existence;
object(1).numberOfGmComponents = componentCount;
object(1).w = (1:componentCount) / sum(1:componentCount);
object(1).mu = cell(1, componentCount);
object(1).Sigma = cell(1, componentCount);
for componentIdx = 1:componentCount
    object(1).mu{componentIdx} = ...
        componentIdx * (1:model.xDimension)';
    factor = diag(componentIdx + (1:model.xDimension));
    object(1).Sigma{componentIdx} = factor * factor';
end
object(1).trajectoryLength = 3;
object(1).trajectory = ones(model.xDimension, 3);
object(1).timestamps = 1:3;
end

function assertTransportFieldsEqual(actual, expected)
assert(numel(actual) == numel(expected));
for objectIdx = 1:numel(expected)
    assert(actual(objectIdx).birthTime == expected(objectIdx).birthTime);
    assert(actual(objectIdx).birthLocation == ...
        expected(objectIdx).birthLocation);
    assert(isequaln(actual(objectIdx).r, expected(objectIdx).r));
    assert(actual(objectIdx).numberOfGmComponents == ...
        expected(objectIdx).numberOfGmComponents);
    assert(isequaln(actual(objectIdx).w, ...
        reshape(expected(objectIdx).w, 1, [])));
    for componentIdx = 1:expected(objectIdx).numberOfGmComponents
        assert(isequaln(actual(objectIdx).mu{componentIdx}, ...
            reshape(expected(objectIdx).mu{componentIdx}, [], 1)));
        expectedCovariance = (expected(objectIdx).Sigma{componentIdx} + ...
            expected(objectIdx).Sigma{componentIdx}') / 2;
        assert(isequaln(actual(objectIdx).Sigma{componentIdx}, ...
            expectedCovariance));
    end
end
end

function assertTemplateFields(objects, model)
assert(isequal(fieldnames(objects), fieldnames(model.object)));
for objectIdx = 1:numel(objects)
    assert(objects(objectIdx).trajectoryLength == 0);
    assert(isequal(size(objects(objectIdx).trajectory), ...
        [model.xDimension, 0]));
    assert(isequal(size(objects(objectIdx).timestamps), [1, 0]));
end
end

function assertMetadata(actual, expected, objectCount, totalBytes)
assert(double(actual.version) == 1);
assert(double(actual.eventType) == expected.eventType);
assert(double(actual.sender) == expected.sender);
assert(double(actual.receiver) == expected.receiver);
assert(double(actual.timeIndex) == expected.timeIndex);
assert(double(actual.objectCount) == objectCount);
assert(double(actual.totalBytes) == totalBytes);
end

function testMalformedDecoderInputs( ...
    emptyBytes, lightBytes, fullBytes, model)
bad = emptyBytes;
bad(1) = 0;
assertThrows(@() decodeLmbWireMessage(bad, model));
bad = emptyBytes;
bad(5) = 2;
assertThrows(@() decodeLmbWireMessage(bad, model));
bad = emptyBytes;
bad(6) = 0;
assertThrows(@() decodeLmbWireMessage(bad, model));
bad = emptyBytes;
bad(7) = bad(7) + 1;
assertThrows(@() decodeLmbWireMessage(bad, model));
bad = emptyBytes;
bad(8) = 1;
assertThrows(@() decodeLmbWireMessage(bad, model));
assertThrows(@() decodeLmbWireMessage(emptyBytes(1:end-1), model));
assertThrows(@() decodeLmbWireMessage([emptyBytes, uint8(0)], model));
assertThrows(@() decodeLmbWireMessage(lightBytes(1:end-1), model));

bad = emptyBytes;
bad(21:24) = uint8([25, 0, 0, 0]);
assertThrows(@() decodeLmbWireMessage(bad, model));
bad = emptyBytes;
bad(17:20) = uint8([255, 255, 255, 255]);
assertThrows(@() decodeLmbWireMessage(bad, model));

bad = fullBytes;
bad(6) = 1;
assertThrows(@() decodeLmbWireMessage(bad, model));
bad = lightBytes;
bad(41:42) = uint8([0, 0]);
assertThrows(@() decodeLmbWireMessage(bad, model));
bad = lightBytes;
bad(6) = 2;
bad(41:42) = uint8([255, 255]);
assertThrows(@() decodeLmbWireMessage(bad, model));
bad = lightBytes;
bad(43) = 1;
assertThrows(@() decodeLmbWireMessage(bad, model));
bad = lightBytes;
bad(45:52) = uint8([0, 0, 0, 0, 0, 0, 248, 127]);
assertThrows(@() decodeLmbWireMessage(bad, model));
bad = lightBytes;
bad(33:40) = uint8([0, 0, 0, 0, 0, 0, 248, 127]);
assertThrows(@() decodeLmbWireMessage(bad, model));
end

function testEncoderValidation(light, full, metadata, model)
bad = light;
bad.birthTime = -1;
assertThrows(@() encodeLmbWireMessage(bad, metadata, model));
bad = light;
bad.birthLocation = 1.5;
assertThrows(@() encodeLmbWireMessage(bad, metadata, model));
bad = light;
bad.r = 1.1;
assertThrows(@() encodeLmbWireMessage(bad, metadata, model));
bad = light;
bad.w = -1;
assertThrows(@() encodeLmbWireMessage(bad, metadata, model));
bad = light;
bad.w = 0;
assertThrows(@() encodeLmbWireMessage(bad, metadata, model));
bad = light;
bad.mu = {[1; 2]};
assertThrows(@() encodeLmbWireMessage(bad, metadata, model));
bad = light;
bad.Sigma = {ones(model.xDimension + 1)};
assertThrows(@() encodeLmbWireMessage(bad, metadata, model));
bad = light;
bad.Sigma{1}(1, 2) = bad.Sigma{1}(1, 2) + 1;
assertThrows(@() encodeLmbWireMessage(bad, metadata, model));
bad = light;
bad.numberOfGmComponents = 2;
assertThrows(@() encodeLmbWireMessage(bad, metadata, model));
bad = light;
bad.numberOfGmComponents = 0;
bad.w = zeros(1, 0);
bad.mu = cell(1, 0);
bad.Sigma = cell(1, 0);
assertThrows(@() encodeLmbWireMessage(bad, metadata, model));
assertThrows(@() encodeLmbWireMessage(full, metadata, model));
bad = light;
bad.mu{1}(1) = NaN;
assertThrows(@() encodeLmbWireMessage(bad, metadata, model));
bad = light;
bad.Sigma{1}(1, 1) = Inf;
assertThrows(@() encodeLmbWireMessage(bad, metadata, model));

badMetadata = metadata;
badMetadata.eventType = 0;
assertThrows(@() encodeLmbWireMessage(light, badMetadata, model));
badMetadata = metadata;
badMetadata.sender = double(intmax('uint16')) + 1;
assertThrows(@() encodeLmbWireMessage(light, badMetadata, model));
badMetadata = metadata;
badMetadata.sender = -1;
assertThrows(@() encodeLmbWireMessage(light, badMetadata, model));
badMetadata = metadata;
badMetadata.receiver = 1.5;
assertThrows(@() encodeLmbWireMessage(light, badMetadata, model));
badMetadata = metadata;
badMetadata.timeIndex = double(intmax('uint32')) + 1;
assertThrows(@() encodeLmbWireMessage(light, badMetadata, model));
badMetadata = metadata;
badMetadata.timeIndex = -1;
assertThrows(@() encodeLmbWireMessage(light, badMetadata, model));
badModel = model;
badModel.xDimension = 256;
assertThrows(@() encodeLmbWireMessage(light, metadata, badModel));
end

function assertThrows(functionHandle)
didThrow = false;
try
    functionHandle();
catch
    didThrow = true;
end
assert(didThrow);
end
