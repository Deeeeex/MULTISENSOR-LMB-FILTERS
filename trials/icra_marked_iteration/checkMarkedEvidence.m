function checkMarkedEvidence()
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
addpath(fullfile(root,'trials','icra_ceiling_iteration'));
checkEvidenceCeiling();
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');
model.T=.1;model.sensorMotionEnabled=false;model.sensorFovEnabled=false;
o=model.birthParameters(1);o.birthTime=1;o.birthLocation=1;o.r=.2;
o.numberOfGmComponents=1;o.w=1;o.mu={zeros(4,1)};o.Sigma={eye(4)};
o.hasObservationLineage=true;o.lastDirectOpportunity=20;o.positiveConfirmation=false;o.directEvidenceCeiling=.8;
p=o;p.r=.8;p.birthLocation=2;p.mu={[4;0;0;0]};
model.birthParameters=o;model.object=o([]);
for Z={{[.1;0],[3.9;.1]}, {}}
    [a,da,Wa]=updateLmbWithAssociationWeights([o,p],Z{1},model,1,20);
    [b,db,Wb]=updateMarkedLmb([o,p],Z{1},model,1,20,ones(1,numel(Z{1})));
    assert(isequaln(a,b) && isequaln(da,db) && isequaln(Wa,Wb));
end
Z={[.1;0]};[A,~]=generateLmbSensorAssociationMatrices(o,Z,model,1);previous=0;
for ratio=[.1,1,10]
    [b,~,W]=updateMarkedLmb(o,Z,model,1,20,ratio);
    expected=(A.phi+A.L(1,2)*ratio)/(A.eta+A.L(1,2)*ratio);
    assert(abs(b.r-expected)<1e-12 && b.r>previous);previous=b.r;
    c=directEvidenceCeilings(W,.8,true);
    assert(c>=0 && c<=b.detectionAssociationMass+1e-12);
end
% Marked No-age wrapper must equal the original fusion rule on fixed inputs.
p.birthLocation=1;p.lastDirectOpportunity=1;
cfg=buildMixtureAwareKlaReferenceConfig();cfg.missingLabelFusionMode='fov-aware-censored';
d=struct('eventType',[0,2],'sourceIndices',[1,2],'isStale',[false,false],'isSelf',[true,false],'currentTime',20);
for r=[.05,.2,.8,.95]
    o.r=r;
    [a,~]=fuseValidationInputs({o,p},[.5,.5],model,d,cfg,'lineage',20);
    [b,~]=fuseMarkedInputs({o,p},[.5,.5],model,d,cfg,'marked_lineage',20);
    assert(abs(a.r-b.r)<1e-12 && isequal(a.mu,b.mu) && isequal(a.Sigma,b.Sigma));
end
fprintf('MARKED CHECK PASSED: exact unity-mark and empty-update parity, scalar Bayes identity and monotonicity, actual association support, original No-age fusion parity.\n');
end
