function [pd,Q,info]=evaluateSensorQuality(model,sensor,target,time)
% Shared replay observation model; covariance and geometric support unchanged.
base=model.replayBaseSensorQuality;
assert(endsWith(functions(base).file,[filesep,'common',filesep,'evaluateSensorQuality.m']));
[pd,Q,info]=base(model,sensor,target,time);
xy=target(1:2);inside=abs(xy(1))<=70.4 && abs(xy(2))<=40;
for n=1:2
    position=model.sensorTrajectories{n}(1:2,time);
    if sum((xy-position).^2)<=9,inside=false;end
end
if ~inside,pd=0;info.inFov=false;end
if pd>0
    mode=model.rangeDetectionMode;fit=model.rangeDetection;
    if strcmp(mode,'range')
        z=fit.intercept+fit.slope*info.range/40;
        pd=1/(1+exp(-z));
    elseif strcmp(mode,'constant'),pd=fit.constant;
    else,assert(strcmp(mode,'nominal'));end
    assert(isfinite(pd) && pd>0 && pd<1);
end
end
