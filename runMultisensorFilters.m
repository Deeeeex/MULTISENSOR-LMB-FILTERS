% RUNMULTISENSORFILTERS - Run the multi-sensor LMB or LMBM filters

%% Admin
close all; clc;
setPath;
%% Select type of filter
filterType = 'PU'; % 'IC', 'PU', 'LMBM'
%% Generate the model
numberOfSensors = 3;
clutterRates = [5 5 5];
detectionProbabilities = [0.67 0.70 0.73];
q = [4 3 2];
model = generateMultisensorModel(numberOfSensors, clutterRates, detectionProbabilities, q, 'PU', 'LBP', 'Fixed');
%% Generate observations
[groundTruth, measurements, groundTruthRfs] = generateMultisensorGroundTruth(model);
%% Apply communication model (optional)
commConfig = struct();
commConfig.level = 1; % 0=ideal, 1=bandwidth, 2=link loss, 3=node outage
commConfig.globalMaxSensorPacketsPerStep = 2;
commConfig.sensorWeights = ones(1, numberOfSensors) / numberOfSensors;
commConfig.priorityPolicy = 'weightedPriority';
commConfig.linkModel = 'fixed';
commConfig.pDrop = 0.2;
commConfig.maxOutageNodes = 1;
[measurementsDelivered, commStats] = applyCommunicationModel(measurements, model, commConfig);
%% Run filters
if(strcmp(filterType, 'IC'))
    % Iterated-corrector LMB (IC-LMB) filter
    stateEstimates = runIcLmbFilter(model, measurementsDelivered);
elseif(strcmp(filterType, 'PU'))
    % Parallel measurement update: PU-, GA-, or AA-LMB filters
    stateEstimates = runParallelUpdateLmbFilter(model, measurementsDelivered);
else
    % Multisensor LMBM filter
    stateEstimates = runMultisensorLmbmFilter(model, measurementsDelivered);
end
%% Plotting
plotMultisensorResults(model, measurementsDelivered, groundTruth, stateEstimates, groundTruthRfs);
 
