function [groundTruth, measurements, groundTruthRfs, sensorTrajectories] = generateMultisensorGroundTruthEnhanced(model, varargin)
% GENERATEMULTISENSORGROUNDTRUTHENHANCED - Enhanced version with extended sensor motion
%   [groundTruth, measurements, groundTruthRfs, sensorTrajectories] = generateMultisensorGroundTruthEnhanced(model)
%
%   Enhanced version supporting CT motion, formation flight, and other Phase 2 features.
%
%   See also generateMultisensorGroundTruth
%
%   Inputs
%       model - struct. A struct with fields declared in generateMultisensorModel.
%       numberOfObjects - integer. The number of objects to be simulated
%           for a 'Random' scenario.
%
%   Output
%       groundTruth - cell array. An array of each object's groundtruth
%           trajectory.
%       measurements - cell array. An array containing measurements for
%           each time-step of the simulation.
%       groundTruthRfs - struct. Groundtruth RFS and optimal Kalman filter
%           output in RFS form.
%       sensorTrajectories - cell array. Each sensor's trajectory over time.

%% Call base function first
[groundTruth, measurements, groundTruthRfs, sensorTrajectories] = generateMultisensorGroundTruth(model, varargin{:});

%% Phase 2: Enhanced Sensor Motion
if model.sensorMotionEnabled
    %% CT (Coordinated Turn) motion model
    if isfield(model, 'sensorMotionType') && strcmp(model.sensorMotionType, 'CT')
        sensorTrajectories = generateCTMotionTrajectories(model);
    end

    %% Formation flight mode
    if isfield(model, 'sensorFormation')
        sensorTrajectories = generateFormationFlightTrajectories(model);
    end

    %% Adaptive velocity
    if isfield(model, 'sensorAdaptiveVelocity') && model.sensorAdaptiveVelocity
        sensorTrajectories = generateAdaptiveSensorVelocities(model, sensorTrajectories);
    end

    %% Sensor scheduling
    if isfield(model, 'sensorScheduling')
        sensorTrajectories = applySensorScheduling(model, sensorTrajectories);
    end
end

end

%% CT Motion Model Generator
function sensorTrajectories = generateCTMotionTrajectories(model)
% GENERATECTMOTIONTRAJECTORIES - Generate trajectories with coordinated turns
    sensorTrajectories = cell(1, model.numberOfSensors);
    simulationLength = 100;

    for s = 1:model.numberOfSensors
        sensorTrajectories{s} = zeros(4, simulationLength);
        sensorTrajectories{s}(:, 1) = model.sensorInitialStates{s};

        omega = 0; % Initial turn rate
        for t = 2:simulationLength
            % Update turn rate (white noise)
            if isfield(model, 'sensorTurnModel') && strcmp(model.sensorTurnModel, 'WhiteNoiseTurn')
                omega = omega + randn() * 0.05;
            else
                omega = omega + (rand() - 0.5) * 0.1;
            end

            % Rotation matrix for turn
            cos_w = cos(model.sensorTurnRate * model.T);
            sin_w = sin(model.sensorTurnRate * model.T);

            % State transition with turn
            prev_state = sensorTrajectories{s}(:, t-1);
            prev_pos = prev_state(1:2);
            prev_vel = prev_state(3:4);

            % CT motion: velocity rotates during the time step
            new_vel = [cos_w * prev_vel(1) - sin_w * prev_vel(2);
                       sin_w * prev_vel(1) + cos_w * prev_vel(2)];

            new_pos = prev_pos + new_vel * model.T;

            % Add process noise
            process_noise = chol(model.sensorProcessNoise{s}, 'lower') * randn(4, 1);

            sensorTrajectories{s}(:, t) = [new_pos; new_vel] + process_noise;
        end
    end
end

%% Formation Flight Generator
function sensorTrajectories = generateFormationFlightTrajectories(model)
% GENERATEFORMATIONFLIGHTTRAJECTORIES - Generate formation flight trajectories
    sensorTrajectories = cell(1, model.numberOfSensors);
    simulationLength = 100;

    % Define formation relative positions
    switch model.sensorFormation
        case 'Line'
            % Line formation: sensors in a straight line
            formationOffsets = {
                [0; 0; 0; 0],
                [0; -10; 0; 0],
                [0; -20; 0; 0]
            };
        case 'Triangle'
            % Triangle formation
            angle = (0:2) * 2 * pi / 3;
            radius = 15;
            for i = 1:min(model.numberOfSensors, 3)
                formationOffsets{i} = [radius * cos(angle(i)); radius * sin(angle(i)); 0; 0];
            end
        case 'Circle'
            % Circular formation
            radius = 20;
            for i = 1:model.numberOfSensors
                angle = 2 * pi * (i-1) / model.numberOfSensors;
                formationOffsets{i} = [radius * cos(angle); radius * sin(angle)); 0; 0];
            end
        otherwise
            % Default: no formation
            for i = 1:model.numberOfSensors
                formationOffsets{i} = [0; 0; 0; 0];
            end
    end

    % Generate trajectories
    centerTraj = zeros(4, simulationLength);
    centerTraj(:, 1) = [0; 0; model.formationSpeed; 0];

    for t = 2:simulationLength
        centerTraj(:, t) = model.A * centerTraj(:, t-1) + ...
            chol((model.sensorProcessNoiseStd^2) * eye(4), 'lower') * randn(4, 1);
    end

    for s = 1:model.numberOfSensors
        sensorTrajectories{s} = zeros(4, simulationLength);
        offset = formationOffsets{min(s, length(formationOffsets))};
        for t = 1:simulationLength
            sensorTrajectories{s}(:, t) = centerTraj(:, t) + [offset(1:2); 0; 0];
        end
    end
end

%% Adaptive Velocity Generator
function sensorTrajectories = generateAdaptiveSensorVelocities(model, sensorTrajectories)
% GENERATEADAPTIVESENSORVELOCITIES - Apply adaptive velocity limits to sensors
    for s = 1:model.numberOfSensors
        traj = sensorTrajectories{s};

        for t = 2:size(traj, 2)
            % Current velocity
            vel = traj(3:4, t);
            speed = norm(vel);

            % Adaptive: scale down if too fast
            if speed > model.maxSensorVelocity
                scale = model.maxSensorVelocity / speed;
                vel = vel * scale;

                % Update position
                pos = traj(1:2, t-1) + vel * model.T;

                % Update trajectory
                traj(1:2, t) = pos;
                traj(3:4, t) = vel;
            end
        end

        sensorTrajectories{s} = traj;
    end
end

%% Sensor Scheduling
function sensorTrajectories = applySensorScheduling(model, sensorTrajectories)
% APPLYSENSORSCHEDULING - Apply scheduling to sensor availability
    sensorTrajectories = cell(1, model.numberOfSensors);
    simulationLength = 100;

    % Initialize with base trajectories
    [~, baseTrajectories] = generateMultisensorGroundTruth(model);

    for t = 1:simulationLength
        activeSensors = model.sensorSchedule(:, t);

        for s = 1:model.numberOfSensors
            if ~activeSensors(s)
                % Sensor inactive: move to hidden position
                sensorTrajectories{s}(:, t) = [9999; 9999; 0; 0];
            else
                sensorTrajectories{s}(:, t) = baseTrajectories{s}(:, t);
            end
        end
    end
end
