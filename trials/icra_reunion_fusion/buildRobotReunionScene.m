function scene=buildRobotReunionScene(name)
% Prescribed low-speed robot patrol geometry; no task-outcome-dependent motion.
if strcmp(name,'churn_departure'), source='boundary_churn';
elseif ismember(name,{'split_latebirth','split_no_new'}), source='split_rejoin';
else, error('Unknown reunion scene'); end
scene=buildGroupPilotScene(source);
scene.name=name;
scene.positions=.4*scene.positions;
for n=1:scene.N, scene.trajectories{n}=.4*scene.trajectories{n}; end
scene.radioRange=24;
scene.priors=[-9.6,9.6,-24.8,24.8,-24.8,24.8; ...
    -1.6,1.6,4.08,4.08,-8,-8;zeros(1,6);.24*ones(1,6)];
scene.birthFrames=[1,1,35,35,35,35];
scene.activeRegions=[1,2,3,4];
if strcmp(name,'split_no_new'), scene.activeRegions=[1,2]; end
scene.departureFrame=inf;
if strcmp(name,'churn_departure'), scene.departureFrame=91; end
end
