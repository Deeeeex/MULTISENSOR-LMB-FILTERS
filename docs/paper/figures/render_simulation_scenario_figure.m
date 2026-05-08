function render_simulation_scenario_figure()
% Render the scenario schematic from the actual paper run configuration.
%
% Left panel: standard fixed ideal benchmark from RUN/IDEAL.
% Right panel: headline 4+4 formation tiered-loss scenario from RUN/GA.

script_dir = fileparts(mfilename('fullpath'));
paper_fig_dir = fullfile(script_dir, '..', 'els-cas-templates', 'figs');
if ~exist(paper_fig_dir, 'dir')
    mkdir(paper_fig_dir);
end

source_pdf = fullfile(script_dir, 'figure_simulation_scenario.pdf');
paper_pdf = fullfile(paper_fig_dir, 'paper-figure-scenario.pdf');

ideal_cfg = buildStandardIdealConfig();
main_cfg = buildMainScenarioConfig();

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1360 590]);
set(fig, 'PaperUnits', 'inches');
set(fig, 'PaperPosition', [0 0 11.7 5.05]);
set(fig, 'PaperSize', [11.7 5.05]);

ax_ideal = axes('Parent', fig, 'Position', [0.055 0.15 0.425 0.76]);
drawStandardIdealPanel(ax_ideal, ideal_cfg);

ax_tiered = axes('Parent', fig, 'Position', [0.545 0.15 0.425 0.76]);
drawMainScenarioPanel(ax_tiered, main_cfg);

print(fig, source_pdf, '-dpdf', '-painters');
print(fig, paper_pdf, '-dpdf', '-painters');
close(fig);
fprintf('Wrote %s\n', source_pdf);
fprintf('Wrote %s\n', paper_pdf);
end

function cfg = buildStandardIdealConfig()
cfg.simLength = 100;
cfg.numberOfSensors = 4;
cfg.clutterRate = 5;
cfg.detectionProbability = 0.9;
cfg.measurementNoiseStd = 3;
cfg.sensorPositions = [-96, 96, 96, -96; 96, 96, -96, -96];
cfg.sensorEdges = [1 2; 2 3; 3 4; 4 1];

cfg.birthTimes = [1, 1, 20, 20, 40, 40, 60, 60, 60, 60];
cfg.deathTimes = [70, 70, 80, 80, 90, 90, 100, 100, 100, 100];
cfg.targetStates = [
    -80.0 -20.0 0.75 1.5;
    -20.0 80.0 -1.0 -2.0;
    0.0 0.0 -0.5 -1.0;
    40.0 -60.0 -0.25 -0.5;
    -80.0 -20.0 1.0 1.0;
    40.0 -60.0 -1.0 2.0;
    -80.0 -20.0 1.0 -0.5;
    -20.0 80.0 1.0 -1.0;
    0.0 0.0 1.0 -1.0;
    40.0 -60.0 -1.0 0.5
]';
end

function cfg = buildMainScenarioConfig()
cfg.simLength = 100;
cfg.sensorVelocity = [0.8; 0];
cfg.fovHalfAngleDeg = 60;
cfg.fovRangeForPlot = 82;
cfg.birthInterval = 8;
cfg.sensorStart = buildSensorInitialStates();
cfg.sensorEnd = cfg.sensorStart;
cfg.sensorEnd(1:2, :) = cfg.sensorEnd(1:2, :) + (cfg.simLength - 1) * cfg.sensorVelocity;
cfg.neighborEdges = buildNeighborEdges4Plus4();

[cfg.targetBirthStates, cfg.targetGroupIndex] = buildTargetBirthStates();
cfg.birthTimes = 1 + (0:size(cfg.targetBirthStates, 2) - 1) * cfg.birthInterval;
cfg.deathTimes = min(cfg.birthTimes + 100 - 1, cfg.simLength);

% Trial 1 from the saved headline reports. The code randomly permutes the
% tier counts each trial, so this is shown only as a representative draw.
cfg.examplePDropBySensor = [0.1 0.1 0.2 0.1 0 0.5 0.5 0.1];
cfg.globalMaxMeasurementsPerStep = 80;
cfg.pDropLevels = [0 0.1 0.2 0.5];
cfg.pDropLevelCounts = [1 4 1 2];
end

function drawStandardIdealPanel(ax, cfg)
setupScenarioAxes(ax, [-112 112], [-112 112]);
title(ax, '(a) Standard fixed ideal scenario', 'FontSize', 11, ...
    'FontWeight', 'bold', 'Color', [0.16 0.19 0.24]);

drawIdealRoi(ax);
drawStandardIdealTargets(ax, cfg);
drawIdealSensorRing(ax, cfg);

text(ax, -94, 101, '4 sensors, Fixed benchmark, ideal communication', ...
    'Interpreter', 'none', 'FontSize', 7.8, 'Color', [0.38 0.42 0.49], ...
    'BackgroundColor', 'w', 'Margin', 1);
text(ax, -94, -103, 'full common ROI, p drop by sensor = [0 0 0 0]', ...
    'Interpreter', 'none', 'FontSize', 7.4, 'Color', [0.38 0.42 0.49], ...
    'BackgroundColor', 'w', 'Margin', 1);
hold(ax, 'off');
end

function drawMainScenarioPanel(ax, cfg)
setupScenarioAxes(ax, [-132 112], [-114 112]);
title(ax, '(b) Main tiered-loss scenario', 'FontSize', 11, ...
    'FontWeight', 'bold', 'Color', [0.16 0.19 0.24]);

subtitleText = sprintf('8 mobile sensors, Mmax=%d, p drop tier counts [1,4,1,2]', ...
    cfg.globalMaxMeasurementsPerStep);

drawSensorFovFans(ax, cfg);
drawTargetTrajectories(ax, cfg);
drawSensorMotion(ax, cfg);
drawNeighborGraph(ax, cfg.neighborEdges, cfg.sensorStart, true);
drawSensors(ax, cfg.sensorStart, cfg.examplePDropBySensor);
drawTierLegend(ax, cfg);

text(ax, -122, 71, 'sensor group A', 'FontSize', 7.8, 'FontWeight', 'bold', ...
    'Color', [0.40 0.44 0.50], 'BackgroundColor', 'w', 'Margin', 1);
text(ax, -122, -69, 'sensor group B', 'FontSize', 7.8, 'FontWeight', 'bold', ...
    'Color', [0.40 0.44 0.50], 'BackgroundColor', 'w', 'Margin', 1);
text(ax, -128, 101, subtitleText, 'Interpreter', 'none', 'FontSize', 7.8, ...
    'Color', [0.38 0.42 0.49], 'BackgroundColor', 'w', 'Margin', 1);
hold(ax, 'off');
end

function setupScenarioAxes(ax, xLimits, yLimits)
hold(ax, 'on');
axis(ax, 'equal');
xlim(ax, xLimits);
ylim(ax, yLimits);
set(ax, 'FontSize', 7.5, 'LineWidth', 0.8, 'XColor', [0.46 0.50 0.56], ...
    'YColor', [0.46 0.50 0.56], 'Box', 'on', 'Layer', 'top');
grid(ax, 'on');
set(ax, 'GridColor', [0.84 0.87 0.91], 'GridAlpha', 0.35);
xlabel(ax, 'x position', 'FontSize', 8.2);
ylabel(ax, 'y position', 'FontSize', 8.2);
end

function drawIdealRoi(ax)
plot(ax, [-100 100 100 -100 -100], [-100 -100 100 100 -100], '--', ...
    'Color', [0.59 0.64 0.70], 'LineWidth', 0.9);
text(ax, 48, -84, 'common ROI', 'FontSize', 7.3, ...
    'Color', [0.40 0.44 0.50], 'BackgroundColor', 'w', 'Margin', 1);
end

function drawStandardIdealTargets(ax, cfg)
birthGroups = unique(cfg.birthTimes);
colors = [
    0.23 0.27 0.33;
    0.13 0.47 0.54;
    0.74 0.38 0.13;
    0.39 0.31 0.58
];
for i = 1:size(cfg.targetStates, 2)
    x0 = cfg.targetStates(:, i);
    duration = cfg.deathTimes(i) - cfg.birthTimes(i);
    x1 = x0 + [duration * x0(3); duration * x0(4); 0; 0];
    p0 = x0(1:2);
    p1 = x1(1:2);
    groupIdx = find(birthGroups == cfg.birthTimes(i), 1);
    color = colors(groupIdx, :);
    plot(ax, [p0(1) p1(1)], [p0(2) p1(2)], '-', 'Color', color, ...
        'LineWidth', 0.95);
    drawArrowHead(ax, p0 + 0.82 * (p1 - p0), p1, color, 3.0);
    plot(ax, p0(1), p0(2), 'o', 'MarkerSize', 3.6, ...
        'MarkerFaceColor', 'w', 'MarkerEdgeColor', color, 'LineWidth', 0.85);
end
text(ax, -6, 84, 'Fixed births: t = 1, 20, 40, 60', ...
    'Interpreter', 'none', 'FontSize', 7.4, 'Color', [0.38 0.42 0.49], ...
    'BackgroundColor', 'w', 'Margin', 1);
text(ax, -6, 75, '10 standard benchmark targets', ...
    'Interpreter', 'none', 'FontSize', 7.4, 'Color', [0.38 0.42 0.49], ...
    'BackgroundColor', 'w', 'Margin', 1);
end

function drawIdealSensorRing(ax, cfg)
for e = 1:size(cfg.sensorEdges, 1)
    a = cfg.sensorEdges(e, 1);
    b = cfg.sensorEdges(e, 2);
    p1 = cfg.sensorPositions(:, a);
    p2 = cfg.sensorPositions(:, b);
    plot(ax, [p1(1) p2(1)], [p1(2) p2(2)], '-', ...
        'Color', [0.34 0.53 0.70], 'LineWidth', 1.0);
end
for s = 1:cfg.numberOfSensors
    p = cfg.sensorPositions(:, s);
    rectangle(ax, 'Position', [p(1) - 5, p(2) - 5, 10, 10], ...
        'Curvature', [1 1], 'FaceColor', [0.88 0.94 0.99], ...
        'EdgeColor', [0.17 0.22 0.29], 'LineWidth', 1.0);
    text(ax, p(1), p(2), sprintf('S%d', s), 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'FontSize', 7.2, 'FontWeight', 'bold', ...
        'Color', [0.14 0.16 0.20]);
end
text(ax, -93, 84, 'conceptual sensor ring', 'FontSize', 7.3, ...
    'Color', [0.40 0.44 0.50], 'BackgroundColor', 'w', 'Margin', 1);
end

function drawSensorFovFans(ax, cfg)
sensorStart = cfg.sensorStart;
range = cfg.fovRangeForPlot;
halfAngle = cfg.fovHalfAngleDeg * pi / 180;
rayColor = [0.78 0.88 0.96];
for s = 1:size(sensorStart, 2)
    p = sensorStart(1:2, s)';
    angles = [-halfAngle, halfAngle];
    for a = 1:numel(angles)
        endpoint = p + range * [cos(angles(a)), sin(angles(a))];
        plot(ax, [p(1) endpoint(1)], [p(2) endpoint(2)], '-', ...
            'Color', rayColor, 'LineWidth', 0.55);
    end
end
end

function drawSensorMotion(ax, cfg)
for s = 1:size(cfg.sensorStart, 2)
    p0 = cfg.sensorStart(1:2, s);
    p1 = cfg.sensorEnd(1:2, s);
    plot(ax, [p0(1) p1(1)], [p0(2) p1(2)], '-', 'Color', [0.48 0.61 0.73], ...
        'LineWidth', 1.0);
    plot(ax, p1(1), p1(2), '.', 'Color', [0.48 0.61 0.73], 'MarkerSize', 8);
end
drawArrowHead(ax, [-18; 89], [0; 89], [0.48 0.61 0.73], 4.2);
text(ax, -78, 90, 'mobile sensors: v = [0.8, 0]', 'Interpreter', 'none', ...
    'FontSize', 7.6, 'Color', [0.38 0.42 0.49], 'BackgroundColor', 'w', 'Margin', 1);
end

function drawNeighborGraph(ax, edges, sensorStart, isTiered)
for e = 1:size(edges, 1)
    a = edges(e, 1);
    b = edges(e, 2);
    p1 = sensorStart(1:2, a);
    p2 = sensorStart(1:2, b);
    if abs(a - b) == 4
        lineStyle = '--';
        width = 1.15;
    else
        lineStyle = '-';
        width = 0.9;
    end
    if isTiered
        color = [0.44 0.50 0.57];
    else
        color = [0.34 0.53 0.70];
    end
    plot(ax, [p1(1) p2(1)], [p1(2) p2(2)], lineStyle, ...
        'Color', color, 'LineWidth', width);
end
end

function drawSensors(ax, sensorStart, pDropBySensor)
for s = 1:size(sensorStart, 2)
    p = sensorStart(1:2, s);
    fillColor = tierColor(pDropBySensor(s));
    rectangle(ax, 'Position', [p(1) - 5, p(2) - 5, 10, 10], ...
        'Curvature', [1 1], 'FaceColor', fillColor, ...
        'EdgeColor', [0.17 0.22 0.29], 'LineWidth', 1.0);
    text(ax, p(1), p(2), sprintf('S%d', s), 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'FontSize', 7.2, 'FontWeight', 'bold', ...
        'Color', [0.14 0.16 0.20]);
end
end

function drawTargetTrajectories(ax, cfg)
colors = [
    0.23 0.27 0.33;
    0.23 0.27 0.33;
    0.23 0.27 0.33
];
for i = 1:size(cfg.targetBirthStates, 2)
    x0 = cfg.targetBirthStates(:, i);
    duration = cfg.deathTimes(i) - cfg.birthTimes(i);
    x1 = x0 + [duration * x0(3); duration * x0(4); 0; 0];
    p0 = x0(1:2);
    p1 = x1(1:2);
    groupIdx = cfg.targetGroupIndex(i);
    color = colors(groupIdx, :);
    plot(ax, [p0(1) p1(1)], [p0(2) p1(2)], '-', 'Color', color, ...
        'LineWidth', 0.85);
    drawArrowHead(ax, p0 + 0.80 * (p1 - p0), p1, color, 3.0);
    plot(ax, p0(1), p0(2), 'o', 'MarkerSize', 3.5, ...
        'MarkerFaceColor', 'w', 'MarkerEdgeColor', color, 'LineWidth', 0.8);
end
text(ax, 48, 94, 'target wave 1: 3', 'FontSize', 7.4, ...
    'Color', [0.34 0.37 0.43], 'BackgroundColor', 'w', 'Margin', 1);
text(ax, 92, 4, 'wave 2: 3', 'FontSize', 7.4, ...
    'Color', [0.34 0.37 0.43], 'BackgroundColor', 'w', 'Margin', 1);
text(ax, 45, -91, 'wave 3: 4', 'FontSize', 7.4, ...
    'Color', [0.34 0.37 0.43], 'BackgroundColor', 'w', 'Margin', 1);
text(ax, 45, 84, sprintf('staggered births every %d steps', cfg.birthInterval), ...
    'Interpreter', 'none', 'FontSize', 7.4, ...
    'Color', [0.38 0.42 0.49], 'BackgroundColor', 'w', 'Margin', 1);
end

function drawTierLegend(ax, cfg)
x0 = -126;
y0 = -103;
labels = {'0', '0.1', '0.2', '0.5'};
text(ax, x0, y0 + 11, 'example p drop by sensor', 'Interpreter', 'none', ...
    'FontSize', 7.3, 'FontWeight', 'bold', 'Color', [0.38 0.42 0.49], ...
    'BackgroundColor', 'w', 'Margin', 1);
for i = 1:numel(cfg.pDropLevels)
    x = x0 + (i - 1) * 30;
    rectangle(ax, 'Position', [x, y0 - 3.2, 7.5, 6.4], 'Curvature', [1 1], ...
        'FaceColor', tierColor(cfg.pDropLevels(i)), 'EdgeColor', [0.17 0.22 0.29], ...
        'LineWidth', 0.7);
    text(ax, x + 9.5, y0, labels{i}, 'Interpreter', 'none', 'FontSize', 7.2, ...
        'Color', [0.38 0.42 0.49], 'VerticalAlignment', 'middle');
end
end

function sensorStates = buildSensorInitialStates()
groupCenters = [-80, -80; 35, -35];
groupSpacing = 20;
sensorsPerGroup = 4;
sensorStates = zeros(4, 8);
idx = 1;
offsets = localFormationOffsets('Leader3', groupSpacing, sensorsPerGroup);
for g = 1:2
    center = groupCenters(:, g);
    for k = 1:sensorsPerGroup
        pos = center + offsets(:, k);
        sensorStates(:, idx) = [pos; 0.8; 0];
        idx = idx + 1;
    end
end
end

function edges = buildNeighborEdges4Plus4()
groupA = 1:4;
groupB = 5:8;
pairings = [1 5; 2 6; 3 7; 4 8];
edges = [];
edges = [edges; allPairs(groupA)];
edges = [edges; allPairs(groupB)];
edges = [edges; pairings];
end

function pairs = allPairs(nodes)
pairs = [];
for i = 1:numel(nodes) - 1
    for j = i + 1:numel(nodes)
        pairs = [pairs; nodes(i), nodes(j)]; %#ok<AGROW>
    end
end
end

function [targetBirthStates, groupIndex] = buildTargetBirthStates()
targetCenter = [0; 0];
groupCenters = [70, 80, 70; 80, 0, -80];
groupTypes = {'Triangle', 'Triangle', 'Leader3'};
groupCounts = [3, 3, 4];
groupSpacing = [30, 25, 20];
groupSpeed = [0.45, 0.45, 0.45];
totalTargets = sum(groupCounts);
targetBirthStates = zeros(4, totalTargets);
groupIndex = zeros(1, totalTargets);
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
        groupIndex(idx) = g;
        idx = idx + 1;
    end
end
end

function offsets = localFormationOffsets(formationType, spacing, count)
switch lower(formationType)
    case 'triangle'
        base = [0, -0.5, 0.5; 0, -0.866, -0.866];
    case 'leader3'
        base = [0, -1, -1, -2; 0, -0.7, 0.7, 0];
    otherwise
        base = [0, -1, 1; 0, -1, -1];
end
if size(base, 2) < count
    base = [base, zeros(2, count - size(base, 2))];
end
offsets = spacing * base(:, 1:count);
end

function color = tierColor(drop)
if drop <= 0
    color = [0.88 0.94 0.99];
elseif drop <= 0.1
    color = [0.86 0.95 0.92];
elseif drop <= 0.2
    color = [0.98 0.90 0.70];
else
    color = [0.97 0.82 0.80];
end
end

function drawArrowHead(ax, pPrev, pEnd, color, scale)
direction = pEnd(:) - pPrev(:);
normDir = sqrt(sum(direction .^ 2));
if normDir < eps
    return;
end
u = direction / normDir;
v = [-u(2); u(1)];
tip = pEnd(:);
base = tip - scale * u;
head = [
    tip';
    (base + 0.55 * scale * v)';
    (base - 0.55 * scale * v)'
];
patch(ax, head(:, 1), head(:, 2), color, 'EdgeColor', color, 'LineWidth', 0.5);
end
