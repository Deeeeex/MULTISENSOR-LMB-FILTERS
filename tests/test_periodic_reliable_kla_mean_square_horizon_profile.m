function test_periodic_reliable_kla_mean_square_horizon_profile()
% Exact phase mapping against direct prefix certificates.

adjacency = false(3, 3, 2);
weights = repmat(eye(3), 1, 1, 2);
reliability = zeros(3, 3, 2);
adjacency(2, 1, 1) = true;
weights(2, 2, 1) = 0.5;
weights(2, 1, 1) = 0.5;
reliability(2, 1, 1) = 0.8;
adjacency(3, 2, 2) = true;
weights(3, 3, 2) = 0.6;
weights(3, 2, 2) = 0.4;
reliability(3, 2, 2) = 0.7;
maximumHorizon = 7;
profile = computePeriodicReliableKlaMeanSquareHorizonProfile( ...
    adjacency, weights, reliability, maximumHorizon, struct( ...
        'targetSquaredFactors', [0.90, 0.50]));
assert(strcmp(profile.contractVersion, ...
    'periodic-reliable-kla-mean-square-horizon-profile-v1'));
assert(profile.period == 2);
assert(profile.allStartPhasesEnumeratedExactly);
for startPhase = 1:2
    for horizon = 1:maximumHorizon
        phases = mod(startPhase - 1 + (0:horizon-1), 2) + 1;
        direct = ...
            computeReliableKlaWindowMeanSquareContractionCertificate( ...
                adjacency(:, :, phases), weights(:, :, phases), ...
                reliability(:, :, phases));
        expected = direct.worstCaseExpectedSquaredContractionFactor;
        actual = profile.factorByStartPhaseAndHorizon( ...
            startPhase, horizon + 1);
        assert(abs(actual - expected) < 2e-12);
        expectedMessages = sum(profile.messageCountsByPhase(phases));
        actualMessages = profile. ...
            cumulativeMessagesByStartPhaseAndHorizon( ...
                startPhase, horizon + 1);
        assert(actualMessages == expectedMessages);
    end
end
assert(isequal(profile.worstStartPhaseFactorByHorizon, ...
    max(profile.factorByStartPhaseAndHorizon, [], 1)));
assert(~profile.posteriorUsed && ~profile.truthUsed && ...
    ~profile.futureOutcomeUsed);
fprintf('PASS: periodic reliable KLA mean-square profile tests\n');
end
