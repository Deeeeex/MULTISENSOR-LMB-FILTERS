function [pd,Q,info]=evaluateSensorQuality(model,sensor,target,time)
% Replay-only observation-domain adapter. The captured handle remains bound
% to the unchanged repository implementation; verify that binding at setup.
base=model.replayBaseSensorQuality;
assert(~contains(functions(base).file,[filesep,'replay_quality',filesep]));
[pd,Q,info]=base(model,sensor,target,time);
xy=target(1:2);inside=abs(xy(1))<=70.4 && abs(xy(2))<=40;
for n=1:2
    position=model.sensorTrajectories{n}(1:2,time);
    if sum((xy-position).^2)<=9,inside=false;end
end
if ~inside,pd=0;info.inFov=false;end
end
