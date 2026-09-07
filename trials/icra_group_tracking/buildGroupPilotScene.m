function scene = buildGroupPilotScene(name)
% Smooth prescribed robot-like paths, independent of targets and results.
dt=0.5; time=(0:119)*dt; N=8;
smooth=@(x) max(0,min(1,x)).^2.*(3-2*max(0,min(1,x)));
if strcmp(name,'split_rejoin')
    separation=40+80*(smooth((time-8)/12)-smooth((time-32)/12));
elseif strcmp(name,'boundary_churn')
    separation=40+smooth((time-6)/6).*(38+8*sin(2*pi*time/6));
else
    error('Unknown scene');
end
positions=zeros(2,N,numel(time)); trajectories=cell(1,N);
signs=[-1,-1,1,1;-1,1,-1,1];
for team=1:2
    for role=1:4
        node=4*(team-1)+role;
        width=12+4*sin(0.5*time+0.4*(team-1));
        height=12-4*sin(0.5*time+0.4*(team-1));
        x=(2*team-3)*separation/2+signs(1,role)*width;
        y=0.6*time+signs(2,role)*height;
        trajectories{node}=[x;y;gradient(x,dt);gradient(y,dt)];
        positions(:,node,:)=reshape([x;y],2,1,[]);
    end
end
scene=struct('name',name,'dt',dt,'time',time,'N',N,'T',numel(time), ...
    'positions',positions,'trajectories',{trajectories},'radioRange',60);
end
