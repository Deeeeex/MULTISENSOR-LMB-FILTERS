function authorization = fusionSufficientInternalTestAuthorization()
% FUSIONSUFFICIENTINTERNALTESTAUTHORIZATION Explicit test-only capability.

stack = dbstack();
isTestCaller = false;
for frameIdx = 1:numel(stack)
    if ~isempty(regexp(stack(frameIdx).name, '^test_', 'once'))
        isTestCaller = true;
        break;
    end
end
if ~isTestCaller
    error('FusionSufficientTestAuthorization:Caller', ...
        'Internal test authorization is available only from test_* code.');
end
authorization = struct( ...
    'schema', 'fusion-sufficient-internal-test-authorization-v1', ...
    'testOnly', true, ...
    'guard', 'never-valid-for-production-evidence');
end
