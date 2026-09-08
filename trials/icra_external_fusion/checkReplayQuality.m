function checkReplayQuality()
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
addpath(fullfile(root,'common'));
base=@evaluateSensorQuality;assert(strcmp(functions(base).file,fullfile(root,'common','evaluateSensorQuality.m')));
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');
model.sensorMotionEnabled=true;model.sensorFovEnabled=true;model.sensorFovRange=40;
model.sensorFovHalfAngleDeg=180;model.sensorQuality=struct('enabled',false);
model.sensorTrajectories={[0;0;0;0],[50;0;0;0]};model.replayBaseSensorQuality=base;
qualityPath=fullfile(out,'replay_quality');addpath(qualityPath);cleanup=onCleanup(@()rmpath(qualityPath)); %#ok<NASGU>
[p0,Q0,~]=base(model,1,[10;0;0;0],1);
[p1,Q1,info]=evaluateSensorQuality(model,1,[10;0;0;0],1);
assert(p0==p1 && isequal(Q0,Q1) && info.inFov);
assert(evaluateSensorQuality(model,1,[0;0;0;0],1)==0);
assert(evaluateSensorQuality(model,2,[75;0;0;0],1)==0);
assert(evaluateSensorQuality(model,1,[45;0;0;0],1)==0);
assert(evaluateSensorQuality(model,2,[48;0;0;0],1)==0);
assert(base(model,1,[0;0;0;0],1)==.9,'Captured base must not recurse into the adapter.');
fprintf('REPLAY QUALITY PASS: unchanged base binding, identical interior likelihood, platform mask, rectangle and source-specific FoV.\n');
end
