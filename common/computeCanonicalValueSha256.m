function digest = computeCanonicalValueSha256(value)
% COMPUTECANONICALVALUESHA256 Hash supported MATLAB/Octave values by content.
%
% Field names are sorted, array sizes and classes are encoded, and double
% values use 17 significant digits. The contract intentionally accepts only
% double, logical, char, cell, and struct values. In particular, integer,
% single, string, complex, sparse, and opaque runtime values fail closed so
% that lossy text conversion cannot make distinct values share a digest.

digest = computeTextSha256(serializeValue(value));
end

function text = serializeValue(value)
if islogical(value)
    if ~isreal(value) || issparse(value)
        error('CanonicalHash:UnsupportedValue', ...
            'Complex and sparse values are not supported.');
    end
    sizeText = sprintf('%d,', size(value));
    valueText = sprintf('%d,', value(:));
    text = sprintf('%s:%s:%s', class(value), sizeText, valueText);
elseif isa(value, 'double')
    if ~isreal(value) || issparse(value)
        error('CanonicalHash:UnsupportedValue', ...
            'Complex and sparse values are not supported.');
    end
    sizeText = sprintf('%d,', size(value));
    valueText = sprintf('%.17g,', value(:));
    text = sprintf('%s:%s:%s', class(value), sizeText, valueText);
elseif ischar(value)
    text = sprintf('char:%s:%s', ...
        sprintf('%d,', size(value)), value(:)');
elseif isnumeric(value) || isstring(value)
    error('CanonicalHash:UnsupportedValue', ...
        'Unsupported value class: %s.', class(value));
elseif iscell(value)
    parts = cell(1, numel(value) + 1);
    parts{1} = sprintf('cell:%s:', sprintf('%d,', size(value)));
    for valueIdx = 1:numel(value)
        parts{valueIdx + 1} = [ ...
            serializeValue(value{valueIdx}), '|'];
    end
    text = [parts{:}];
elseif isstruct(value)
    names = sort(fieldnames(value));
    parts = cell(1, 1 + numel(value) * numel(names));
    parts{1} = sprintf('struct:%s:%s:', ...
        sprintf('%d,', size(value)), strjoin(names, ','));
    partIdx = 1;
    for valueIdx = 1:numel(value)
        for fieldIdx = 1:numel(names)
            partIdx = partIdx + 1;
            fieldName = names{fieldIdx};
            parts{partIdx} = [fieldName, '=', ...
                serializeValue(value(valueIdx).(fieldName)), '|'];
        end
    end
    text = [parts{1:partIdx}];
else
    error('CanonicalHash:UnsupportedValue', ...
        'Unsupported value class: %s.', class(value));
end
end
