function context = ...
    buildFormationBackboneBundleM24SourceExecutionContext( ...
        presetName, seed)
% BUILDFORMATIONBACKBONEBUNDLEM24SOURCEEXECUTIONCONTEXT Opaque permit handle.

permit = getFormationBackboneBundleM24SourceCasePermit( ...
    presetName, seed);
context = struct();
context.contractVersion = ...
    'dynamic-topology-case-execution-context-v1';
context.permitId = permit.id;
context.permitCanonicalSha256 = permit.permitCanonicalSha256;
context.capability = permit.capability;
context.action = permit.filterAction;
end
