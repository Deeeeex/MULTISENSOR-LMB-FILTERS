function checkTemperedRecency()
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
addpath(fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'), ...
    fullfile(root,'trials','icra_reunion_fusion'),fullfile(root,'trials','icra_external_fusion'), ...
    fullfile(root,'trials','icra_ceiling_iteration'),fullfile(root,'trials','icra_marked_iteration'));
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');model.T=.1;
o=model.birthParameters(1);o.birthTime=1;o.birthLocation=1;o.r=.9;
o.numberOfGmComponents=1;o.w=1;o.mu={zeros(4,1)};o.Sigma={eye(4)};
o.hasObservationLineage=true;o.lastDirectOpportunity=20;o.positiveConfirmation=false;o.directEvidenceCeiling=.6;
p=o;p.r=.1;p.lastDirectOpportunity=1;p.directEvidenceCeiling=1;
model.birthParameters=o;model.object=o([]);cfg=buildMixtureAwareKlaReferenceConfig();
cfg.missingLabelFusionMode='fov-aware-censored';cfg.captureIterationRecords=true;
d=struct('eventType',[0,2],'sourceIndices',[1,2],'isStale',[false,false],'isSelf',[true,false],'currentTime',20);
[base,~]=fuseMarkedInputsStable({o,p},[.5,.5],model,d,cfg,'lineage',20);
[er,~]=fuseMarkedInputsStable({o,p},[.5,.5],model,d,cfg,'er',20);
last=-1;
for c=[0,.2,.5,.8,1]
    o.directEvidenceCeiling=c;
    [actual,stats]=fuseTemperedRecency({o,p},[.5,.5],model,d,cfg,'tempered_calibrated',20);
    z0=log(base.r)-log1p(-base.r);zE=log(er.r)-log1p(-er.r);
    expected=1/(1+exp(-((1-c)*z0+c*zE)));
    assert(abs(actual.r-expected)<1e-12 && actual.r>=last);last=actual.r;
    assert(isequal(actual.mu,base.mu) && isequal(actual.Sigma,base.Sigma));
    assert(actual.r>=base.r-1e-14 && actual.r<=er.r+1e-14);
    assert(abs(stats.records(24)-(zE-z0))<1e-12);
end
% Stale or downweighted support must not grant positive age authority.
o.directEvidenceCeiling=0;
[actual,~]=fuseTemperedRecency({o,p},[.5,.5],model,d,cfg,'tempered_calibrated',20);
assert(abs(actual.r-base.r)<1e-12);
o.lastDirectOpportunity=19;o.directEvidenceCeiling=1;
[actual,stats]=fuseTemperedRecency({o,p},[.5,.5],model,d,cfg,'tempered_calibrated',20);
assert(abs(actual.r-base.r)<1e-12 && all(stats.records(20:21)==0));
% Equal ages, one source, untouched-source exclusion, and missing censor.
o.lastDirectOpportunity=20;p.lastDirectOpportunity=20;
[actual,~]=fuseTemperedRecency({o,p},[.5,.5],model,d,cfg,'tempered_calibrated',20);
assert(abs(actual.r-base.r)<1e-12);
[actual,~]=fuseTemperedRecency({o,p},[1,0],model,d,cfg,'tempered_calibrated',20);
assert(abs(actual.r-o.r)<1e-12);
p.hasObservationLineage=false;
[actual,stats]=fuseTemperedRecency({o,p},[.5,.5],model,d,cfg,'tempered_calibrated',20);
assert(abs(actual.r-o.r)<1e-12 && stats.lineageExcluded==1);
[actual,stats]=fuseTemperedRecency({o,model.object},[.5,.5],model,d,cfg,'tempered_calibrated',20);
assert(stats.observableAbsences==1 && isfinite(actual.r));
% Negative shifts remain exactly ER for all support values.
p.hasObservationLineage=true;p.lastDirectOpportunity=1;o.r=.1;p.r=.9;
[er,~]=fuseMarkedInputsStable({o,p},[.5,.5],model,d,cfg,'er',20);
for c=[0,.2,.5,.8,1]
    o.directEvidenceCeiling=c;
    [actual,~]=fuseTemperedRecency({o,p},[.5,.5],model,d,cfg,'tempered_calibrated',20);
    assert(actual.r==er.r);
end
% The ER adapter retains all old fields; support remains a real packet field.
[actual,sa]=fuseTemperedRecency({o,p},[.5,.5],model,d,cfg,'er',20);
[expected,se]=fuseMarkedInputsStable({o,p},[.5,.5],model,d,cfg,'er',20);
assert(isequaln(actual,expected) && isequaln(sa,se));
[decoded,bytes]=ceilingLmbPacket([o,p],model,1,20);
assert(numel(bytes)==32+2*216 && isequal([decoded.directEvidenceCeiling],[1,1]));
assert(isequal(directEvidenceCeilings([],[],[true,false]),[0,0]));
fprintf('TEMPERED CHECK PASSED: monotone geometric interpolation and endpoints, unchanged spatial density and negative correction, equal-age and single-source identity, stale/excluded-source control, censor, packet, and exact ER adapter parity.\n');
end
