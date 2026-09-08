function [count,confirmed]=updateLocalConfirmation(previousCount,previousFrame,t,mass,opportunity,hasMeasurements)
% Current local evidence only: a received posterior cannot create a hit.
count=0;
if opportunity && hasMeasurements && mass>=.5
    count=1;
    if previousFrame==t-1,count=min(2,previousCount+1);end
end
confirmed=count>=2;
end
