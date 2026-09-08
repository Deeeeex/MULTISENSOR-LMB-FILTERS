function checkInnovationRecency()
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
addpath(fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'), ...
    fullfile(root,'trials','icra_reunion_fusion'),fullfile(root,'trials','icra_external_fusion'));
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');model.T=.1;
o=model.birthParameters(1);o.birthTime=1;o.birthLocation=1;o.r=.2;
o.numberOfGmComponents=1;o.w=1;o.mu={zeros(4,1)};o.Sigma={eye(4)};
o.hasObservationLineage=true;o.lastDirectOpportunity=20;o.positiveConfirmation=false;
o.localLogOddsIncrement=-1.7;model.birthParameters=o;model.object=o([]);
p=o;p.r=.9;p.mu={[2;0;0;0]};p.lastDirectOpportunity=1;p.localLogOddsIncrement=0;
cfg=buildMixtureAwareKlaReferenceConfig();cfg.captureIterationRecords=true;
details=struct('eventType',[0,2],'sourceIndices',[1,2],'isStale',[false,false], ...
    'isSelf',[true,false],'currentTime',20);
[ir,stats]=fuseInnovationRecency({o,p},[.5,.5],model,details,cfg,'innovation_recency',20);
q=1/(1+.25+.75*exp(-1.9/5));
base=.5*(log(.2/.8)+log(.9/.1))-.5;
expected=1/(1+exp(-(base+(q-.5)*(-1.7))));
assert(abs(ir.r-expected)<1e-12 && norm(ir.mu{1}-[1;0;0;0])<1e-12);
assert(norm(ir.Sigma{1}-eye(4),'fro')<1e-12 && abs(stats.records(11)+.5)<1e-12);
% No current local information must not amplify inherited disagreement.
o.localLogOddsIncrement=0;
[zero,~]=fuseInnovationRecency({o,p},[.5,.5],model,details,cfg,'innovation_recency',20);
[baseOut,~]=fuseValidationInputs({o,p},[.5,.5],model,details,cfg,'lineage',20);
assert(abs(zero.r-baseOut.r)<1e-12 && isequal(zero.mu,baseOut.mu));
% Equal ages recover ordinary KLA even with unequal current increments.
o.localLogOddsIncrement=-1.7;p.lastDirectOpportunity=20;
[equal,~]=fuseInnovationRecency({o,p},[.5,.5],model,details,cfg,'innovation_recency',20);
assert(abs(equal.r-baseOut.r)<1e-12);
% One eligible source is an identity, including unequal metadata.
[single,~]=fuseInnovationRecency({o,p},[1,0],model,details,cfg,'innovation_recency',20);
assert(abs(single.r-o.r)<1e-12 && isequal(single.mu,o.mu));
% Untouched priors abstain and cannot contribute a large fabricated increment.
p.hasObservationLineage=false;p.localLogOddsIncrement=100;
[excluded,d]=fuseInnovationRecency({o,p},[.5,.5],model,details,cfg,'innovation_recency',20);
assert(abs(excluded.r-o.r)<1e-12 && d.lineageExcluded==1);
% Original ER instrumentation follows the original function.
p.hasObservationLineage=true;p.lastDirectOpportunity=1;p.localLogOddsIncrement=0;
[old,~]=fuseValidationInputs({o,p},[.5,.5],model,details,cfg,'qualified_exist',20);
[instrumented,~]=fuseInnovationRecency({o,p},[.5,.5],model,details,cfg,'qualified_exist',20);
assert(isequal(old.r,instrumented.r) && isequal(old.mu,instrumented.mu) && isequal(old.Sigma,instrumented.Sigma));
[decoded,bytes]=innovationLmbPacket([o,p],model,1,20);
assert(numel(bytes)==32+2*216 && isequal([decoded.localLogOddsIncrement],[-1.7,0]));
[empty,bytes]=innovationLmbPacket(model.object,model,1,20);
assert(isempty(empty) && numel(bytes)==32);
fprintf('IR CHECK PASSED: analytic Gaussian overlap, prediction/innovation separation, zero information, equal ages, identity, untouched exclusion, original ER parity and packet round trip.\n');
end
