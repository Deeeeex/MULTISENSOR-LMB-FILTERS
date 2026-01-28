% RUNMULTISENSORFILTERS_FORMATION - Formation sensors + formation targets demo
close all; clc;
setPath;

%% Switch: staggered target births
staggeredBirths = true; % true = staggered births, false = all at once
makeGif = true;
gifPath = 'formation_animation.gif';
frameSkip = 1;
frameDelay = 0.08;
fovHalfAngleDeg = 60;
fovRange = 60;
fovColor = [0.7 0.7 0.7];
useDistributedFusion = true;
leaderSensor = 1;
sensorCommRange = 50;
fusionWeighting = 'Metropolis'; % 'Metropolis' or 'Uniform'
compareAdaptiveWeights = true;
adaptiveFusionConfig = struct('enabled', true, 'emaAlpha', 0.7, 'minWeight', 0.05);
gifPathBase = 'formation_animation_base.gif';
gifPathAdaptive = 'formation_animation_adaptive.gif';
%% Sensor configuration
numberOfSensors = 5;
clutterRates = [3 3 3 3 3];
detectionProbabilities = [0.9 0.9 0.9 0.9 0.9];
q = [3 3 3 3 3];

commConfig = struct();
commConfig.level = 1; % default Level 1: bandwidth only
commConfig.globalMaxMeasurementsPerStep = 35;
commConfig.sensorWeights = ones(1, numberOfSensors) / numberOfSensors;
commConfig.priorityPolicy = 'weightedPriority';
commConfig.measurementSelectionPolicy = 'random';

%% Sensor formation (Leader+4) moving left-to-right
sensorMotionConfig = struct();
sensorMotionConfig.enabled = true;
sensorMotionConfig.motionType = 'Formation';
sensorMotionConfig.processNoiseStd = 0.05;
sensorMotionConfig.formationType = 'Leader4';
sensorMotionConfig.formationSpacing = 20;
sensorMotionConfig.formationCenterStart = [-80; 0];
sensorMotionConfig.formationVelocity = [0.8; 0];

%% Target formations (3-3-4), all fly toward center
targetFormationConfig = struct();
targetFormationConfig.targetFormationEnabled = true;
targetFormationConfig.targetFormationStaggeredBirths = staggeredBirths;
targetFormationConfig.targetFormationBirthInterval = 8;
targetFormationConfig.targetFormationStartTime = 1;
targetFormationConfig.targetFormationLifeSpan = 100;
targetFormationConfig.targetBirthStates = buildTargetBirthStates();
targetFormationConfig.targetFormationCount = size(targetFormationConfig.targetBirthStates, 2);

%% Generate model with formation scenario
model = generateMultisensorModel(numberOfSensors, clutterRates, ...
    detectionProbabilities, q, 'GA', 'LBP', 'Formation', ...
    sensorMotionConfig, targetFormationConfig);
model.sensorCommRange = sensorCommRange;
model.fusionWeighting = fusionWeighting;
model.adaptiveFusion = adaptiveFusionConfig;

%% Generate observations
[groundTruth, measurements, groundTruthRfs, sensorTrajectories] = generateMultisensorGroundTruth(model);
model.sensorTrajectories = sensorTrajectories;
% Apply communication model (Level 1 by default)
[measurementsDelivered, commStats] = applyCommunicationModel(measurements, model, commConfig);

%% Run filter (distributed local fusion)
stateEstimatesBySensor = [];
localModels = [];
neighborMap = [];
stateEstimatesBase = [];
stateEstimatesAdaptive = [];
if useDistributedFusion
    if compareAdaptiveWeights
        modelBase = model;
        modelBase.adaptiveFusion.enabled = false;
        [stateEstimatesBySensorBase, localModelsBase, neighborMap] = runDistributedLmbFilter( ...
            modelBase, measurementsDelivered, sensorTrajectories, [], commStats);

        modelAdaptive = model;
        modelAdaptive.adaptiveFusion.enabled = true;
        [stateEstimatesBySensorAdaptive, localModelsAdaptive, neighborMap] = runDistributedLmbFilter( ...
            modelAdaptive, measurementsDelivered, sensorTrajectories, neighborMap, commStats);

        stateEstimatesBySensor = stateEstimatesBySensorAdaptive;
        localModels = localModelsAdaptive;
        stateEstimatesBase = stateEstimatesBySensorBase{leaderSensor};
        stateEstimatesAdaptive = stateEstimatesBySensorAdaptive{leaderSensor};
        stateEstimates = stateEstimatesAdaptive;
    else
        [stateEstimatesBySensor, localModels, neighborMap] = runDistributedLmbFilter( ...
            model, measurementsDelivered, sensorTrajectories, [], commStats);
        stateEstimates = stateEstimatesBySensor{leaderSensor};
    end
else
    if compareAdaptiveWeights
        modelBase = model;
        modelBase.adaptiveFusion.enabled = false;
        stateEstimatesBase = runParallelUpdateLmbFilter(modelBase, measurementsDelivered, commStats, sensorTrajectories);

        modelAdaptive = model;
        modelAdaptive.adaptiveFusion.enabled = true;
        stateEstimatesAdaptive = runParallelUpdateLmbFilter(modelAdaptive, measurementsDelivered, commStats, sensorTrajectories);
        stateEstimates = stateEstimatesAdaptive;
    else
        stateEstimates = runParallelUpdateLmbFilter(model, measurementsDelivered, commStats, sensorTrajectories);
    end
end

%% Compute OSPA
[eOspa, hOspa] = computeSimulationOspa(model, groundTruthRfs, stateEstimates);
fprintf('Average E-OSPA: %.3f\n', mean(eOspa));
fprintf('Average H-OSPA: %.3f\n', mean(hOspa));
if useDistributedFusion
    fprintf('=====================================\n');
    fprintf('Per-Sensor OSPA (local fusion)\n');
    fprintf('=====================================\n');
    if compareAdaptiveWeights
        for s = 1:numberOfSensors
            [eBase, hBase] = computeSimulationOspa(localModelsBase{s}, groundTruthRfs, stateEstimatesBySensorBase{s});
            [eAda, hAda] = computeSimulationOspa(localModelsAdaptive{s}, groundTruthRfs, stateEstimatesBySensorAdaptive{s});
            fprintf('Sensor %d: E-OSPA %.3f -> %.3f (%.3f), H-OSPA %.3f -> %.3f (%.3f), neighbors=%s\n', ...
                s, mean(eBase), mean(eAda), mean(eAda) - mean(eBase), ...
                mean(hBase), mean(hAda), mean(hAda) - mean(hBase), mat2str(neighborMap{s}));
        end
    else
        for s = 1:numberOfSensors
            [eOspaLocal, hOspaLocal] = computeSimulationOspa(localModels{s}, groundTruthRfs, stateEstimatesBySensor{s});
            fprintf('Sensor %d: E-OSPA=%.3f, H-OSPA=%.3f, neighbors=%s\n', ...
                s, mean(eOspaLocal), mean(hOspaLocal), mat2str(neighborMap{s}));
        end
    end
end

%% Consensus metrics (distributed only)
if useDistributedFusion
    if compareAdaptiveWeights
        [posBase, cardBase] = computeConsensusMetrics(stateEstimatesBySensorBase, modelBase);
        [posAda, cardAda] = computeConsensusMetrics(stateEstimatesBySensorAdaptive, modelAdaptive);
        fprintf('=====================================\n');
        fprintf('Consensus Metrics (base -> adaptive)\n');
        fprintf('=====================================\n');
        fprintf('Position consensus: %.3f -> %.3f (%.3f)\n', mean(posBase), mean(posAda), mean(posAda) - mean(posBase));
        fprintf('Cardinality consensus: %.3f -> %.3f (%.3f)\n', mean(cardBase), mean(cardAda), mean(cardAda) - mean(cardBase));
        plotConsensusMetrics(posBase, cardBase, posAda, cardAda);
    else
        [posCons, cardCons] = computeConsensusMetrics(stateEstimatesBySensor, model);
        fprintf('=====================================\n');
        fprintf('Consensus Metrics\n');
        fprintf('=====================================\n');
        fprintf('Position consensus: %.3f\n', mean(posCons));
        fprintf('Cardinality consensus: %.3f\n', mean(cardCons));
        plotConsensusMetrics(posCons, cardCons, [], []);
    end
end

%% Plotting
if useDistributedFusion
    measurementsLeader = measurementsDelivered;
    for s = 1:numberOfSensors
        if ~ismember(s, neighborMap{leaderSensor})
            for t = 1:size(measurementsLeader, 2)
                measurementsLeader{s, t} = {};
            end
        end
    end
    plotMultisensorResults(model, measurementsLeader, groundTruth, stateEstimates, groundTruthRfs);
else
    plotMultisensorResults(model, measurementsDelivered, groundTruth, stateEstimates, groundTruthRfs);
end

figure('Position', [100, 100, 1200, 500]);
subplot(1, 2, 1);
hold on; grid on; axis equal;
for i = 1:length(groundTruth)
    traj = groundTruth{i};
    plot(traj(2, :), traj(3, :), 'b-', 'LineWidth', 1.5);
    plot(traj(2, 1), traj(3, 1), 'bo', 'MarkerFaceColor', 'b');
end
for s = 1:numberOfSensors
    sensorTraj = sensorTrajectories{s};
    plot(sensorTraj(1, :), sensorTraj(2, :), 'k--', 'LineWidth', 2);
    plot(sensorTraj(1, 1), sensorTraj(2, 1), 'k^', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
end
for s = 1:numberOfSensors
    sensorTraj = sensorTrajectories{s};
    plotFovRays(sensorTraj(1:2, end), sensorTraj(3:4, end), fovHalfAngleDeg, fovRange, fovColor);
end
xlabel('X Position');
ylabel('Y Position');
title('Ground Truth (Targets) and Sensor Formation');

subplot(1, 2, 2);
hold on; grid on; axis equal;
for i = 1:length(stateEstimates.objects)
    obj = stateEstimates.objects(i);
    if obj.trajectoryLength > 0
        idx = 1:obj.trajectoryLength;
        xTraj = obj.trajectory(1, idx);
        yTraj = obj.trajectory(2, idx);
        plot(xTraj, yTraj, 'r-', 'LineWidth', 1.5);
    end
end
for s = 1:numberOfSensors
    sensorTraj = sensorTrajectories{s};
    plot(sensorTraj(1, :), sensorTraj(2, :), 'k--', 'LineWidth', 2);
end
for s = 1:numberOfSensors
    sensorTraj = sensorTrajectories{s};
    plotFovRays(sensorTraj(1:2, end), sensorTraj(3:4, end), fovHalfAngleDeg, fovRange, fovColor);
end
xlabel('X Position');
ylabel('Y Position');
title(sprintf('Estimated Trajectories (E-OSPA=%.3f)', mean(eOspa)));

%% Local helpers
if makeGif
    if compareAdaptiveWeights && ~isempty(stateEstimatesBase) && ~isempty(stateEstimatesAdaptive)
        animateScenario(model, groundTruth, stateEstimatesBase, sensorTrajectories, gifPathBase, frameSkip, frameDelay, fovHalfAngleDeg, fovRange, fovColor);
        animateScenario(model, groundTruth, stateEstimatesAdaptive, sensorTrajectories, gifPathAdaptive, frameSkip, frameDelay, fovHalfAngleDeg, fovRange, fovColor);
        fprintf('Saved animation (base) to %s\n', gifPathBase);
        fprintf('Saved animation (adaptive) to %s\n', gifPathAdaptive);
    else
        animateScenario(model, groundTruth, stateEstimates, sensorTrajectories, gifPath, frameSkip, frameDelay, fovHalfAngleDeg, fovRange, fovColor);
        fprintf('Saved animation to %s\n', gifPath);
    end
end

function y = smooth1d(x, windowSize)
    if nargin < 2
        windowSize = 5;
    end
    if numel(x) < 3 || windowSize < 2
        y = x;
        return;
    end
    windowSize = min(windowSize, numel(x));
    w = ones(1, windowSize) / windowSize;
    y = conv(x, w, 'same');
end

%% Local helpers
function targetBirthStates = buildTargetBirthStates()
    targetCenter = [0; 0];
    groupCenters = [70, 80, 70; 80, 0, -80]; % right-up, right, right-down
    groupTypes = {'Triangle', 'Triangle', 'Leader3'};
    groupCounts = [3, 3, 4];
    groupSpacing = [30, 25, 20];
    groupSpeed = [0.45, 0.45, 0.45];
    totalTargets = sum(groupCounts);
    targetBirthStates = zeros(4, totalTargets);
    idx = 1;
    for g = 1:numel(groupCounts)
        offsets = localFormationOffsets(groupTypes{g}, groupSpacing(g), groupCounts(g));
        center = groupCenters(:, g);
        dir = targetCenter - center;
        if norm(dir) < 1e-6
            dir = [-1; 0];
        end
        vel = (groupSpeed(g) / norm(dir)) * dir;
        for k = 1:groupCounts(g)
            pos = center + offsets(:, k);
            targetBirthStates(:, idx) = [pos; vel];
            idx = idx + 1;
        end
    end
end

function animateScenario(model, groundTruth, stateEstimates, sensorTrajectories, gifPath, frameSkip, frameDelay, fovHalfAngleDeg, fovRange, fovColor)
    simLength = numel(stateEstimates.mu);
    fig = figure('Position', [150, 150, 900, 600]);
    for t = 1:frameSkip:simLength
        clf(fig);
        hold on; grid on; axis equal;
        xlim(model.observationSpaceLimits(1, :));
        ylim(model.observationSpaceLimits(2, :));
        title(sprintf('t = %d', t));
        xlabel('X Position');
        ylabel('Y Position');

        % Ground truth trajectories up to t
        for i = 1:length(groundTruth)
            traj = groundTruth{i};
            idx = traj(1, :) <= t;
            if any(idx)
                plot(traj(2, idx), traj(3, idx), 'b-', 'LineWidth', 1.0);
            end
        end

        % Estimated trajectories up to t
        for i = 1:length(stateEstimates.objects)
            obj = stateEstimates.objects(i);
            if obj.trajectoryLength > 0
                idx = obj.timestamps(1:obj.trajectoryLength) <= t;
                if any(idx)
                    plot(obj.trajectory(1, idx), obj.trajectory(2, idx), '-', ...
                        'Color', [0.85 0.2 0.2], 'LineWidth', 1.0);
                end
            end
        end

        % Sensor trajectories up to t
        for s = 1:model.numberOfSensors
            sensorTraj = sensorTrajectories{s};
            plot(sensorTraj(1, 1:t), sensorTraj(2, 1:t), 'k--', 'LineWidth', 1.2);
            plot(sensorTraj(1, t), sensorTraj(2, t), 'k^', 'MarkerSize', 7, 'MarkerFaceColor', 'k');
            plotFovRays(sensorTraj(1:2, t), sensorTraj(3:4, t), fovHalfAngleDeg, fovRange, fovColor);
        end

        % Ground truth targets at t
        truthPos = collectTruthAtTime(groundTruth, t);
        if ~isempty(truthPos)
            plot(truthPos(1, :), truthPos(2, :), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6);
        end

        % Estimates at t
        estPos = collectEstimatesAtTime(stateEstimates, t);
        if ~isempty(estPos)
            plot(estPos(1, :), estPos(2, :), 'rx', 'MarkerSize', 7, 'LineWidth', 1.5);
        end

        drawnow;
        frame = getframe(fig);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        if t == 1
            imwrite(imind, cm, gifPath, 'gif', 'Loopcount', inf, 'DelayTime', frameDelay);
        else
            imwrite(imind, cm, gifPath, 'gif', 'WriteMode', 'append', 'DelayTime', frameDelay);
        end
    end
    fprintf('Saved animation to %s\n', gifPath);
end

function [posConsensus, cardConsensus] = computeConsensusMetrics(stateEstimatesBySensor, model)
    numSensors = numel(stateEstimatesBySensor);
    simLength = numel(stateEstimatesBySensor{1}.mu);
    posConsensus = zeros(1, simLength);
    cardConsensus = zeros(1, simLength);
    for t = 1:simLength
        counts = zeros(1, numSensors);
        for s = 1:numSensors
            counts(s) = numel(stateEstimatesBySensor{s}.mu{t});
        end
        medCount = median(counts);
        cardConsensus(t) = mean(abs(counts - medCount));
        pairSum = 0;
        pairCount = 0;
        for i = 1:numSensors-1
            for j = i+1:numSensors
                pairSum = pairSum + estimateSetOspa(stateEstimatesBySensor{i}, stateEstimatesBySensor{j}, t, model);
                pairCount = pairCount + 1;
            end
        end
        if pairCount > 0
            posConsensus(t) = pairSum / pairCount;
        end
    end
end

function dist = estimateSetOspa(estA, estB, t, model)
    muA = estA.mu{t};
    SigmaA = estA.Sigma{t};
    muB = estB.mu{t};
    SigmaB = estB.Sigma{t};
    if isempty(muA) && isempty(muB)
        dist = 0;
        return;
    end
    if isempty(muA) || isempty(muB)
        dist = model.ospaParameters.eC;
        return;
    end
    [eAB, ~] = ospa(muA, muA, SigmaA, muB, SigmaB, model.ospaParameters);
    [eBA, ~] = ospa(muB, muB, SigmaB, muA, SigmaA, model.ospaParameters);
    dist = 0.5 * (eAB(1) + eBA(1));
end

function plotConsensusMetrics(posBase, cardBase, posAda, cardAda)
    figure('Position', [200, 200, 1100, 400]);
    subplot(1, 2, 1);
    hold on; grid on;
    plot(posBase, 'LineWidth', 1.5, 'Color', [0.2 0.2 0.8]);
    if ~isempty(posAda)
        plot(posAda, 'LineWidth', 1.5, 'Color', [0.85 0.2 0.2]);
        legend('Base', 'Adaptive', 'Location', 'best');
    else
        legend('Consensus', 'Location', 'best');
    end
    title('Position Consensus (Pairwise OSPA)');
    xlabel('Time Step');
    ylabel('OSPA');

    subplot(1, 2, 2);
    hold on; grid on;
    plot(cardBase, 'LineWidth', 1.5, 'Color', [0.2 0.2 0.8]);
    if ~isempty(cardAda)
        plot(cardAda, 'LineWidth', 1.5, 'Color', [0.85 0.2 0.2]);
        legend('Base', 'Adaptive', 'Location', 'best');
    else
        legend('Consensus', 'Location', 'best');
    end
    title('Cardinality Consensus (MAD)');
    xlabel('Time Step');
    ylabel('Mean Abs Deviation');
end

function truthPos = collectTruthAtTime(groundTruth, t)
    truthPos = [];
    for i = 1:length(groundTruth)
        traj = groundTruth{i};
        idx = find(traj(1, :) == t, 1, 'first');
        if ~isempty(idx)
            truthPos(:, end+1) = traj(2:3, idx); %#ok<AGROW>
        end
    end
end

function estPos = collectEstimatesAtTime(stateEstimates, t)
    estPos = [];
    if t <= numel(stateEstimates.mu)
        mu = stateEstimates.mu{t};
        if ~isempty(mu)
            for k = 1:numel(mu)
                estPos(:, end+1) = mu{k}(1:2); %#ok<AGROW>
            end
        end
    end
end

function plotFovRays(pos, vel, halfAngleDeg, range, color)
    if nargin < 5
        color = [0.7 0.7 0.7];
    end
    if nargin < 4
        range = 60;
    end
    if nargin < 3
        halfAngleDeg = 60;
    end
    if isempty(pos) || isempty(vel)
        return;
    end
    v = vel(:);
    if norm(v) < 1e-6
        return;
    end
    v = v / norm(v);
    baseAngle = atan2(v(2), v(1));
    a1 = baseAngle + deg2rad(halfAngleDeg);
    a2 = baseAngle - deg2rad(halfAngleDeg);
    p1 = pos + range * [cos(a1); sin(a1)];
    p2 = pos + range * [cos(a2); sin(a2)];
    line([pos(1) p1(1)], [pos(2) p1(2)], 'LineStyle', '--', 'Color', color, 'LineWidth', 1);
    line([pos(1) p2(1)], [pos(2) p2(2)], 'LineStyle', '--', 'Color', color, 'LineWidth', 1);
end

function offsets = localFormationOffsets(formationType, spacing, count)
    if nargin < 3
        count = 3;
    end
    if nargin < 2
        spacing = 12;
    end
    if nargin < 1 || isempty(formationType)
        formationType = 'Triangle';
    end
    formationType = upper(formationType);
    switch formationType
        case 'TRIANGLE'
            radius = spacing / sqrt(3);
            angles = (0:2) * 2 * pi / 3;
            baseOffsets = radius * [cos(angles); sin(angles)];
        case 'LINE'
            idx = (0:count-1) - (count-1)/2;
            baseOffsets = [zeros(1, count); idx * spacing];
        case 'LEADER3'
            y1 = 0.7 * spacing;
            baseOffsets = [0, -spacing, -spacing, -2*spacing;
                           0,  y1,       -y1,       0];
        case 'LEADER4'
            y1 = 0.7 * spacing;
            y2 = 1.4 * spacing;
            baseOffsets = [0, -spacing, -spacing, -2*spacing, -2*spacing;
                           0,  y1,       -y1,       y2,        -y2];
        case 'CIRCLE'
            angles = 2 * pi * (0:count-1) / max(count, 1);
            baseOffsets = spacing * [cos(angles); sin(angles)];
        otherwise
            baseOffsets = zeros(2, max(count, 1));
    end
    if size(baseOffsets, 2) >= count
        offsets = baseOffsets(:, 1:count);
    else
        reps = ceil(count / size(baseOffsets, 2));
        offsets = repmat(baseOffsets, 1, reps);
        offsets = offsets(:, 1:count);
    end
end
