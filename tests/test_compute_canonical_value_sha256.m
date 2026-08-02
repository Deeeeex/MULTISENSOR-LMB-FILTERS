function test_compute_canonical_value_sha256()
% TEST_COMPUTECANONICALVALUESHA256 Stable structured hashing tests.

left = struct('b', {{[1, NaN], logical([1, 0])}}, 'a', 3.25);
right = struct('a', 3.25, 'b', {{[1, NaN], logical([1, 0])}});
leftHash = computeCanonicalValueSha256(left);
rightHash = computeCanonicalValueSha256(right);
assert(strcmp(leftHash, rightHash));

right.b{1}(1) = 2;
assert(~strcmp(leftHash, computeCanonicalValueSha256(right)));

assertUnsupported(single(1));
assertUnsupported(uint64(2^53));
assertUnsupported(uint64(2^53 + 1));
if exist('string', 'builtin') || exist('string', 'file')
    assertUnsupported(string('typed text'));
end

assertUnsupported(@sin);

fprintf('PASS: canonical value SHA-256 tests\n');
end

function assertUnsupported(value)
failed = false;
try
    computeCanonicalValueSha256(value);
catch err
    failed = strcmp(err.identifier, ...
        'CanonicalHash:UnsupportedValue');
end
assert(failed);
end
