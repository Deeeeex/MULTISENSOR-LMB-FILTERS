function checkEvidenceCeiling()
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
addpath(fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'), ...
    fullfile(root,'trials','icra_reunion_fusion'),fullfile(root,'trials','icra_external_fusion'));
% Conditional association weighting includes the zero-mark missed branch.
v=directEvidenceCeilings([.2,.3,.5;.1,.8,.1],[.2,.9],[true,true]);
assert(max(abs(v-[.51,.25]))<1e-14);
assert(isequal(directEvidenceCeilings([.2,.3,.5],[1,1],false),0));
assert(isequal(directEvidenceCeilings([],[],[true,false]),[0,0]));
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');model.T=.1;
o=model.birthParameters(1);o.birthTime=1;o.birthLocation=1;o.r=.9;
o.numberOfGmComponents=1;o.w=1;o.mu={zeros(4,1)};o.Sigma={eye(4)};
o.hasObservationLineage=true;o.lastDirectOpportunity=20;o.positiveConfirmation=false;o.directEvidenceCeiling=.6;
p=o;p.r=.1;p.lastDirectOpportunity=1;p.directEvidenceCeiling=1;
model.birthParameters=o;model.object=o([]);cfg=buildMixtureAwareKlaReferenceConfig();
cfg.missingLabelFusionMode='fov-aware-censored';cfg.captureIterationRecords=true;
d=struct('eventType',[0,2],'sourceIndices',[1,2],'isStale',[false,false],'isSelf',[true,false],'currentTime',20);
[base,~]=fuseValidationInputs({o,p},[.5,.5],model,d,cfg,'lineage',20);
[er,~]=fuseValidationInputs({o,p},[.5,.5],model,d,cfg,'qualified_exist',20);
for c=[0,.51,.6,1]
    o.directEvidenceCeiling=c;
    [actual,~]=fuseEvidenceCeiling({o,p},[.5,.5],model,d,cfg,'ceiling_calibrated',20);
    assert(abs(actual.r-min(er.r,max(base.r,c)))<1e-12);
    assert(isequal(actual.mu,base.mu) && isequal(actual.Sigma,base.Sigma));
end
% Stale/downweighted support cannot authorize a positive age correction.
o.directEvidenceCeiling=0;
[actual,~]=fuseEvidenceCeiling({o,p},[.5,.5],model,d,cfg,'ceiling_calibrated',20);
assert(abs(actual.r-base.r)<1e-12);
% Equal ages, a single source, untouched-source exclusion and absent censor.
p.lastDirectOpportunity=20;
[actual,~]=fuseEvidenceCeiling({o,p},[.5,.5],model,d,cfg,'ceiling_calibrated',20);
assert(abs(actual.r-base.r)<1e-12);
[actual,~]=fuseEvidenceCeiling({o,p},[1,0],model,d,cfg,'ceiling_calibrated',20);
assert(abs(actual.r-o.r)<1e-12);
p.hasObservationLineage=false;
[actual,stats]=fuseEvidenceCeiling({o,p},[.5,.5],model,d,cfg,'ceiling_calibrated',20);
assert(abs(actual.r-o.r)<1e-12 && stats.lineageExcluded==1);
[actual,stats]=fuseEvidenceCeiling({o,model.object},[.5,.5],model,d,cfg,'ceiling_calibrated',20);
assert(stats.observableAbsences==1 && isfinite(actual.r));
% A fresh low-existence source retains the full negative correction.
p.hasObservationLineage=true;p.lastDirectOpportunity=1;o.r=.1;p.r=.9;
[er,~]=fuseValidationInputs({o,p},[.5,.5],model,d,cfg,'qualified_exist',20);
[actual,~]=fuseEvidenceCeiling({o,p},[.5,.5],model,d,cfg,'ceiling_calibrated',20);
assert(actual.r==er.r);
[instrumented,~]=fuseEvidenceCeiling({o,p},[.5,.5],model,d,cfg,'qualified_exist',20);
assert(instrumented.r==er.r && isequal(instrumented.mu,er.mu) && isequal(instrumented.Sigma,er.Sigma));
[decoded,bytes]=ceilingLmbPacket([o,p],model,1,20);
assert(numel(bytes)==32+2*216 && isequal([decoded.directEvidenceCeiling],[0,1]));
[decoded,bytes]=ceilingLmbPacket(model.object,model,1,20);
assert(isempty(decoded) && numel(bytes)==32);
% Exposing W must leave the original local filter and diagnostics exact.
p.birthLocation=2;p.mu={[4;0;0;0]};
for measurements={{[.1;0],[3.9;.1]}, {}}
    Z=measurements{1};
    [original,da]=updateLmbWithSensorMeasurement([o,p],Z,model,1,20);
    [adapter,db,W]=updateLmbWithAssociationWeights([o,p],Z,model,1,20);
    assert(isequaln(original,adapter) && isequaln(da,db));
    if isempty(Z),assert(isempty(W));else,assert(isequal(size(W),[2,3]));end
end
fprintf('ECR CHECK PASSED: association weighting, miss/opportunity reset, continuous bound, unchanged negative correction and spatial density, one/equal-age identity, source exclusion, censor, packet, exact local-update and ER parity.\n');
end
