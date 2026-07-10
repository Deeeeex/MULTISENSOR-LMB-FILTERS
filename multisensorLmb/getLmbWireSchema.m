function schema = getLmbWireSchema()
% GETLMBWIRESCHEMA Versioned application-layer LMB message schema.

schema = struct();
schema.magic = uint8([hex2dec('4c'), hex2dec('4d'), ...
    hex2dec('42'), hex2dec('57')]); % LMBW
schema.version = uint8(1);
schema.lightEventType = uint8(1);
schema.fullEventType = uint8(2);
schema.flags = uint8(3); % bit 0: packed upper triangle; bit 1: binary64
schema.headerBytes = 24;
schema.objectHeaderBytes = 20;
schema.floatBytes = 8;
schema.covarianceSymmetryTolerance = 1e-10;
schema.maxStateDimension = double(intmax('uint8'));
schema.maxSensorIndex = double(intmax('uint16'));
schema.maxTimeIndex = double(intmax('uint32'));
schema.maxObjectCount = double(intmax('uint32'));
schema.maxComponentCount = double(intmax('uint16'));
schema.maxTotalBytes = double(intmax('uint32'));
end
