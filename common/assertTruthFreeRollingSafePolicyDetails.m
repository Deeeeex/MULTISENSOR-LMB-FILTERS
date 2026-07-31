function assertTruthFreeRollingSafePolicyDetails(details)
% ASSERTTRUTHFREEROLLINGSAFEPOLICYDETAILS Preserve truth provenance.

if ~isstruct(details)
    error('RollingSafeCausalOption:InvalidTruthProvenance', ...
        'Rolling-safe policy details must be a structure.');
end
requiredFields = {'truthUsed', 'actionSequenceTruthUsed'};
for fieldIdx = 1:numel(requiredFields)
    fieldName = requiredFields{fieldIdx};
    if ~isfield(details, fieldName)
        error('RollingSafeCausalOption:InvalidTruthProvenance', ...
            'Rolling-safe policy details lack %s.', fieldName);
    end
    value = requireFiniteBinaryDiagnostic( ...
        details.(fieldName), 1, fieldName);
    if value
        error('RollingSafeCausalOption:TruthUse', ...
            'Rolling-safe causal option reports %s=true.', fieldName);
    end
end

optionalFields = {'groundTruthUsed', 'futureOutcomeUsed'};
for fieldIdx = 1:numel(optionalFields)
    fieldName = optionalFields{fieldIdx};
    if ~isfield(details, fieldName)
        continue;
    end
    value = requireFiniteBinaryDiagnostic( ...
        details.(fieldName), 1, fieldName);
    if value
        error('RollingSafeCausalOption:TruthUse', ...
            'Rolling-safe causal option reports %s=true.', fieldName);
    end
end
end
