function checkRangeDetection(baseQuality)
% Analytic one-Bernoulli missed/detected updates verify actual adapter routing.
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');
model.sensorMotionEnabled=true;model.sensorFovEnabled=true;
model.sensorFovRange=40;model.sensorFovHalfAngleDeg=180;
model.sensorQuality=struct('enabled',false);
model.sensorTrajectories={[0;0;0;0],[60;0;0;0]};
model.replayBaseSensorQuality=baseQuality;
model.rangeDetection=struct('intercept',3.4857627462594856,'slope',-3.1072756940997053,'constant',.8136912428684921);
model.clutterPerUnitVolume=[3,3]/(pi*40^2);
for mode={'nominal','range','constant'}
    model.rangeDetectionMode=mode{1};
    for distance=[5,20,39,41]
        o=model.birthParameters(1);o.r=.4;o.mu={[distance;0;0;0]};
        o.w=1;o.numberOfGmComponents=1;o.Sigma={diag([4,4,9,9])};
        expected=.9;
        if strcmp(mode{1},'range'),expected=1/(1+exp(-(model.rangeDetection.intercept+model.rangeDetection.slope*distance/40)));
        elseif strcmp(mode{1},'constant'),expected=model.rangeDetection.constant;end
        if distance>40,expected=0;end
        [actual,Q]=evaluateSensorQuality(model,1,o.mu{1},1);
        assert(abs(actual-expected)<1e-14 && isequal(Q,model.Q{1}));
        missed=updateMarkedLmbStable(o,{},model,1,1,[]);
        phi=o.r*(1-expected);eta=1-o.r+phi;
        assert(abs(missed.r-phi/eta)<1e-13);
        z=[distance+.2;.1];ratio=2;
        [hit,~,W]=updateMarkedLmbStable(o,{z},model,1,1,ratio);
        S=o.Sigma{1}(1:2,1:2)+Q;innovation=z-o.mu{1}(1:2);
        likelihood=exp(-.5*(innovation'*(S\innovation)))/(2*pi*sqrt(det(S)));
        term=o.r*max(expected,realmin)*likelihood*ratio/model.clutterPerUnitVolume(1);
        assert(abs(hit.r-(phi+term)/(eta+term))<1e-12);
        assert(abs(W(1)-phi/(phi+term))<1e-12);
    end
    for xy={[0;0],[60;0],[71;0],[0;41]}
        [actual,~,info]=evaluateSensorQuality(model,1,[xy{1};0;0],1);
        assert(actual==0 && ~info.inFov);
    end
end
fprintf('RANGE DETECTION CHECK PASSED\n');
end
