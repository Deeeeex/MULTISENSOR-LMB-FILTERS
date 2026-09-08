function checkStableMarkedEvidence()
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
    for scale=[.1,1,10]
        ratios=scale*ones(1,numel(Z{1}));
        [a,~,Wa]=updateMarkedLmb([o,p],Z{1},model,1,20,ratios);
        [b,db,Wb]=updateMarkedLmbStable([o,p],Z{1},model,1,20,ratios);
        assert(isequaln(a,b) && isequaln(Wa,Wb));
        if scale==1 || isempty(Z{1})
            [original,diagnostics,W]=updateLmbWithAssociationWeights([o,p],Z{1},model,1,20);
            assert(isequaln(original,b) && isequaln(diagnostics,db) && isequaln(W,Wb));
        end
    end
end
Z={[.1;0]};[A,~]=generateLmbSensorAssociationMatrices(o,Z,model,1);
for ratio=[.1,1,10]
    [b,~,~]=updateMarkedLmbStable(o,Z,model,1,20,ratio);
    expected=(A.phi+A.L(1,2)*ratio)/(A.eta+A.L(1,2)*ratio);
    assert(abs(b.r-expected)<1e-12);
end
p.birthLocation=1;p.lastDirectOpportunity=1;
cfg=buildMixtureAwareKlaReferenceConfig();cfg.missingLabelFusionMode='fov-aware-censored';cfg.captureIterationRecords=true;
d=struct('eventType',[0,2],'sourceIndices',[1,2],'isStale',[false,false],'isSelf',[true,false],'currentTime',20);
for arm={'marked_lineage','marked_er','marked_ceiling_association','marked_ceiling_score','marked_ceiling_calibrated'}
    for r=[.05,.2,.8,.95]
        o.r=r;
        [a,da]=fuseMarkedInputs({o,p},[.5,.5],model,d,cfg,arm{1},20);
        [b,db]=fuseMarkedInputsStable({o,p},[.5,.5],model,d,cfg,arm{1},20);
        assert(isequaln(a,b) && isequaln(da,db));
    end
end
fprintf('STABLE MARKED CHECK PASSED: exact prior-adapter state/W parity, original unity/empty diagnostic parity, unchanged scalar Bayes, exact original fusion-object and record parity for all five arms.\n');
end
