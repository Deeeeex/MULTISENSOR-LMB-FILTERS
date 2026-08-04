function context = ...
    buildFormationB4V46DevelopmentArmExecutionContext( ...
        caseOrdinal, armId)
% BUILDFORMATIONB4V46DEVELOPMENTARMEXECUTIONCONTEXT Opaque permit handle.

permit = loadFormationB4V46DevelopmentArmPermit(caseOrdinal, armId);
context = struct();
context.contractVersion = ...
    'formation-b4-v46-development-arm-execution-context-v1';
context.permitId = permit.id;
context.permitCanonicalSha256 = permit.permitCanonicalSha256;
context.capability = permit.capability;
context.action = permit.filterAction;
context.caseOrdinal = permit.caseOrdinal;
context.armId = permit.armId;
end
