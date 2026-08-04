function test_formation_b4_v47_runtime_evidence_verifier()
% Fail closed when a V47 policy coherently tampers with runtime evidence.

global formationB4V47EvidenceTamperMode;
[model, measurements, sensorTrajectories, neighborMap, ...
    commConfig, identity] = buildFormationB4RuntimeSmokeCase();
model.simulationLength = 1;
measurements = measurements(:, 1);
for sensorIdx = 1:numel(sensorTrajectories)
    sensorTrajectories{sensorIdx} = ...
        sensorTrajectories{sensorIdx}(:, 1);
end
commConfig.pDropByEdge = commConfig.pDropByEdge(:, :, 1);
[commConfig.linkUniforms, ~] = ...
    materializePhysicalUidDirectedDeliveryUniforms( ...
        47008, identity.sensorPhysicalUids, 1);
protocol = getFormationGatewayDebtV47Protocol();
config = buildFormationB4V47FixedTriggerConfig( ...
    protocol.primaryArms{2}, model.numberOfSensors);
config.topologyPolicyFcn = @tamperedV47Policy;

modes = { ...
    'coherent-selected-uid-tamper', ...
    'coherent-service-horizon-tamper', ...
    'causal-flag-tamper', ...
    'projection-hash-tamper', ...
    'route-hash-tamper'};
for modeIdx = 1:numel(modes)
    formationB4V47EvidenceTamperMode = modes{modeIdx};
    assertErrorId(@() runEventTriggeredDistributedLmbFilter( ...
        model, measurements, sensorTrajectories, neighborMap, ...
        commConfig, config), ...
        'FormationB4V47:RuntimeEvidenceDrift');
end
formationB4V47EvidenceTamperMode = [];
fprintf('PASS: FormationB4V47 runtime evidence verifier tests\n');
end

function [adjacency, details] = tamperedV47Policy(context)
global formationB4V47EvidenceTamperMode;
[adjacency, details] = ...
    selectFormationB4V47GatewayDebtRuntimePolicy(context);
switch formationB4V47EvidenceTamperMode
    case 'coherent-selected-uid-tamper'
        changed = details.selectedReceiverPhysicalUids;
        changed(1) = changed(1) + 100000;
        details.selectedReceiverPhysicalUids = changed;
        details.scheduleCertificate. ...
            selectedReceiverPhysicalUids = changed;
        details.scheduleCanonicalSha256 = ...
            computeCanonicalValueSha256( ...
                details.scheduleCertificate);
    case 'coherent-service-horizon-tamper'
        details.crossServiceHorizon = 100;
        details.scheduleCertificate.crossServiceHorizon = 100;
        details.scheduleCanonicalSha256 = ...
            computeCanonicalValueSha256( ...
                details.scheduleCertificate);
    case 'causal-flag-tamper'
        details.pastSuccessfulDeliveryUsed = false;
        details.scheduleCertificate. ...
            pastSuccessfulDeliveryUsed = false;
        details.scheduleCanonicalSha256 = ...
            computeCanonicalValueSha256( ...
                details.scheduleCertificate);
    case 'projection-hash-tamper'
        details.projectionCertificateCanonicalSha256 = ...
            repmat('0', 1, 64);
    case 'route-hash-tamper'
        details.routeAndWeightCanonicalSha256 = repmat('0', 1, 64);
    otherwise
        error('FormationB4V47EvidenceTest:UnknownTamper', ...
            'Unknown V47 evidence tamper mode.');
end
end

function assertErrorId(action, expectedId)
didFail = false;
try
    action();
catch errorInfo
    didFail = true;
    assert(strcmp(errorInfo.identifier, expectedId), ...
        'Expected %s, received %s.', ...
        expectedId, errorInfo.identifier);
end
assert(didFail, 'Expected %s to be raised.', expectedId);
end
