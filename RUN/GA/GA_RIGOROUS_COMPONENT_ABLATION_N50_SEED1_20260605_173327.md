# GA Rigorous Component Ablation (2026-06-05 17:33:27)

Comparison order: Fixed Metropolis -> Cov only -> Link only -> Exist only -> Cov + Link -> Cov + Link + Exist, shared weights -> Cov + Link + Exist, branch-decoupled -> + structure-aware -> + EMA/floor -> + FID-FIA existence only

## Run Config
- Trials: 50
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51]
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- linkModel: fixed
- pDrop target mean: 0.200
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

- finalArmMode: rigorousComponentAblation

## Arm Configs
### Fixed Metropolis
- purpose: Fixed topology-weight baseline.
- enabled: 0
- method: factorized
- useCovariance: 0
- useLinkQuality: 0
- useExistenceConfidence: 0
- useHistorySmoothedExistenceConfidence: 0
- existenceHistoryEmaAlpha: 0.800
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 0
- useStructureAwareKla: 0
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- emaAlpha: 0.000
- minWeight: 0.000
- spatialEmaAlpha: 0.000
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.000
- existenceMinWeight: 0.000
- spatialDecouplingStrength: 0.000
- existenceDecouplingStrength: 0.000
- spatialStructureStrength: 0.000
- existenceStructureStrength: 0.000
- structureReliabilityPower: 0.000
- structureReliabilityMinScore: 0.250
- useFidFiaExistence: 0

### Cov only
- purpose: Isolate posterior concentration.
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 0
- useExistenceConfidence: 0
- useHistorySmoothedExistenceConfidence: 0
- existenceHistoryEmaAlpha: 0.800
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 0
- useStructureAwareKla: 0
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- emaAlpha: 0.000
- minWeight: 0.000
- spatialEmaAlpha: 0.000
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.000
- existenceMinWeight: 0.000
- spatialDecouplingStrength: 0.000
- existenceDecouplingStrength: 0.000
- spatialStructureStrength: 0.000
- existenceStructureStrength: 0.000
- structureReliabilityPower: 0.000
- structureReliabilityMinScore: 0.250
- useFidFiaExistence: 0

### Link only
- purpose: Isolate realized communication quality.
- enabled: 1
- method: factorized
- useCovariance: 0
- useLinkQuality: 1
- useExistenceConfidence: 0
- useHistorySmoothedExistenceConfidence: 0
- existenceHistoryEmaAlpha: 0.800
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 0
- useStructureAwareKla: 0
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- emaAlpha: 0.000
- minWeight: 0.000
- spatialEmaAlpha: 0.000
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.000
- existenceMinWeight: 0.000
- spatialDecouplingStrength: 0.000
- existenceDecouplingStrength: 0.000
- spatialStructureStrength: 0.000
- existenceStructureStrength: 0.000
- structureReliabilityPower: 0.000
- structureReliabilityMinScore: 0.250
- useFidFiaExistence: 0

### Exist only
- purpose: Isolate Bernoulli existence confidence under shared scalar weights.
- enabled: 1
- method: factorized
- useCovariance: 0
- useLinkQuality: 0
- useExistenceConfidence: 1
- useHistorySmoothedExistenceConfidence: 0
- existenceHistoryEmaAlpha: 0.800
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 0
- useStructureAwareKla: 0
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- emaAlpha: 0.000
- minWeight: 0.000
- spatialEmaAlpha: 0.000
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.000
- existenceMinWeight: 0.000
- spatialDecouplingStrength: 0.000
- existenceDecouplingStrength: 0.000
- spatialStructureStrength: 0.000
- existenceStructureStrength: 0.000
- structureReliabilityPower: 0.000
- structureReliabilityMinScore: 0.250
- useFidFiaExistence: 0

### Cov + Link
- purpose: Measure the covariance-link adaptive backbone.
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 0
- useHistorySmoothedExistenceConfidence: 0
- existenceHistoryEmaAlpha: 0.800
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 0
- useStructureAwareKla: 0
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- emaAlpha: 0.000
- minWeight: 0.000
- spatialEmaAlpha: 0.000
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.000
- existenceMinWeight: 0.000
- spatialDecouplingStrength: 0.000
- existenceDecouplingStrength: 0.000
- spatialStructureStrength: 0.000
- existenceStructureStrength: 0.000
- structureReliabilityPower: 0.000
- structureReliabilityMinScore: 0.250
- useFidFiaExistence: 0

### Cov + Link + Exist, shared weights
- purpose: Test existence confidence before branch decoupling.
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useHistorySmoothedExistenceConfidence: 0
- existenceHistoryEmaAlpha: 0.800
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 0
- useStructureAwareKla: 0
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- emaAlpha: 0.000
- minWeight: 0.000
- spatialEmaAlpha: 0.000
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.000
- existenceMinWeight: 0.000
- spatialDecouplingStrength: 0.000
- existenceDecouplingStrength: 0.000
- spatialStructureStrength: 0.000
- existenceStructureStrength: 0.000
- structureReliabilityPower: 0.000
- structureReliabilityMinScore: 0.250
- useFidFiaExistence: 0

### Cov + Link + Exist, branch-decoupled
- purpose: Isolate branch-specific spatial and existence weights.
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useHistorySmoothedExistenceConfidence: 0
- existenceHistoryEmaAlpha: 0.800
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 1
- useStructureAwareKla: 0
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- emaAlpha: 0.000
- minWeight: 0.000
- spatialEmaAlpha: 0.000
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.000
- existenceMinWeight: 0.000
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.000
- existenceStructureStrength: 0.000
- structureReliabilityPower: 0.000
- structureReliabilityMinScore: 0.250
- useFidFiaExistence: 0

### + structure-aware
- purpose: Isolate the weak topology and communication-reliability prior.
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useHistorySmoothedExistenceConfidence: 0
- existenceHistoryEmaAlpha: 0.800
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 1
- useStructureAwareKla: 1
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- emaAlpha: 0.000
- minWeight: 0.000
- spatialEmaAlpha: 0.000
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.000
- existenceMinWeight: 0.000
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- structureReliabilityPower: 0.300
- structureReliabilityMinScore: 0.250
- useFidFiaExistence: 0

### + EMA/floor
- purpose: Isolate temporal smoothing and final-weight protection.
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useHistorySmoothedExistenceConfidence: 0
- existenceHistoryEmaAlpha: 0.800
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 1
- useStructureAwareKla: 1
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- emaAlpha: 0.700
- minWeight: 0.050
- spatialEmaAlpha: 0.700
- existenceEmaAlpha: 0.700
- spatialMinWeight: 0.050
- existenceMinWeight: 0.050
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- structureReliabilityPower: 0.300
- structureReliabilityMinScore: 0.250
- useFidFiaExistence: 0

### + FID-FIA existence only
- purpose: Isolate FID-FIA on the existence branch while retaining stabilization.
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useHistorySmoothedExistenceConfidence: 0
- existenceHistoryEmaAlpha: 0.800
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 1
- useStructureAwareKla: 1
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- emaAlpha: 0.700
- minWeight: 0.050
- spatialEmaAlpha: 0.700
- existenceEmaAlpha: 0.700
- spatialMinWeight: 0.050
- existenceMinWeight: 0.050
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- structureReliabilityPower: 0.300
- structureReliabilityMinScore: 0.250
- useFidFiaExistence: 1
- fidFiaExistenceStrength: 4.000
- fidFiaExistenceMinScore: 0.000
- fidFiaUseExistenceWeight: 1
- fidFiaExistencePower: 1.000
- fidFiaQuadraturePoints: 3
- fidFiaUseDetectionProbability: 1
- fidFiaUseEma: 0
- fidFiaMinWeight: 0.000

## Per-Trial pDropBySensor
- Trial 1: [0.1 0.5 0.2 0 0.5 0.1 0.1 0.1]
- Trial 2: [0.1 0.1 0.2 0.5 0.1 0 0.1 0.5]
- Trial 3: [0.5 0.1 0.1 0.2 0.1 0 0.1 0.5]
- Trial 4: [0.2 0.1 0.1 0.1 0 0.5 0.1 0.5]
- Trial 5: [0.1 0.1 0.1 0.1 0.5 0 0.5 0.2]
- Trial 6: [0.1 0 0.5 0.5 0.1 0.2 0.1 0.1]
- Trial 7: [0 0.1 0.5 0.2 0.1 0.1 0.1 0.5]
- Trial 8: [0 0.5 0.1 0.1 0.5 0.1 0.2 0.1]
- Trial 9: [0.1 0 0.5 0.1 0.1 0.5 0.1 0.2]
- Trial 10: [0 0.5 0.5 0.1 0.1 0.1 0.2 0.1]
- Trial 11: [0.5 0.2 0.1 0.1 0.1 0.5 0 0.1]
- Trial 12: [0.1 0.2 0.1 0 0.5 0.5 0.1 0.1]
- Trial 13: [0.5 0.5 0 0.1 0.1 0.1 0.1 0.2]
- Trial 14: [0 0.1 0.1 0.1 0.5 0.2 0.5 0.1]
- Trial 15: [0.5 0 0.1 0.2 0.1 0.1 0.5 0.1]
- Trial 16: [0.1 0.2 0.1 0.1 0.1 0.5 0 0.5]
- Trial 17: [0.5 0.1 0.1 0.5 0.1 0 0.1 0.2]
- Trial 18: [0.5 0.1 0.1 0.1 0.5 0 0.1 0.2]
- Trial 19: [0.1 0.1 0.1 0.1 0.2 0 0.5 0.5]
- Trial 20: [0.1 0.2 0.1 0.1 0.5 0.1 0.5 0]
- Trial 21: [0.1 0.1 0.5 0.5 0.2 0.1 0.1 0]
- Trial 22: [0.1 0 0.1 0.1 0.1 0.5 0.2 0.5]
- Trial 23: [0 0.1 0.1 0.2 0.1 0.1 0.5 0.5]
- Trial 24: [0.1 0.2 0 0.5 0.1 0.5 0.1 0.1]
- Trial 25: [0.2 0.5 0.1 0 0.1 0.5 0.1 0.1]
- Trial 26: [0.5 0.1 0.1 0.5 0.1 0 0.2 0.1]
- Trial 27: [0 0.5 0.1 0.1 0.1 0.1 0.2 0.5]
- Trial 28: [0 0.5 0.1 0.1 0.1 0.2 0.1 0.5]
- Trial 29: [0.5 0.5 0.1 0 0.1 0.1 0.1 0.2]
- Trial 30: [0.5 0 0.1 0.1 0.1 0.1 0.2 0.5]
- Trial 31: [0 0.5 0.1 0.1 0.2 0.1 0.5 0.1]
- Trial 32: [0.1 0.5 0.2 0.1 0 0.5 0.1 0.1]
- Trial 33: [0.5 0.1 0.5 0 0.2 0.1 0.1 0.1]
- Trial 34: [0.2 0.5 0.1 0.5 0.1 0.1 0 0.1]
- Trial 35: [0.5 0.2 0.5 0.1 0.1 0.1 0.1 0]
- Trial 36: [0.1 0.5 0.1 0.1 0.1 0 0.2 0.5]
- Trial 37: [0.1 0.1 0.5 0.5 0.2 0.1 0.1 0]
- Trial 38: [0.1 0.1 0.1 0.5 0.5 0 0.1 0.2]
- Trial 39: [0.1 0.2 0 0.1 0.5 0.1 0.1 0.5]
- Trial 40: [0.5 0.1 0.1 0.5 0.1 0 0.1 0.2]
- Trial 41: [0.1 0.5 0.1 0.2 0.1 0 0.1 0.5]
- Trial 42: [0.5 0.1 0.1 0.1 0.2 0.1 0.5 0]
- Trial 43: [0.5 0 0.1 0.1 0.2 0.5 0.1 0.1]
- Trial 44: [0.1 0.2 0.1 0.1 0 0.5 0.5 0.1]
- Trial 45: [0.1 0.5 0.5 0.1 0.2 0 0.1 0.1]
- Trial 46: [0.1 0.1 0.2 0 0.1 0.1 0.5 0.5]
- Trial 47: [0 0.1 0.1 0.1 0.2 0.1 0.5 0.5]
- Trial 48: [0.1 0.1 0.1 0.5 0.2 0 0.5 0.1]
- Trial 49: [0 0.5 0.5 0.1 0.2 0.1 0.1 0.1]
- Trial 50: [0.1 0.5 0.1 0.2 0.1 0.5 0 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 2 | Fixed Metropolis | 2.362792 | 3.167487 | 0.772500 |
| 1 | 2 | Cov only | 2.082371 | 1.972368 | 0.555000 |
| 1 | 2 | Link only | 1.815467 | 1.618127 | 0.137500 |
| 1 | 2 | Exist only | 2.260672 | 2.462464 | 0.748750 |
| 1 | 2 | Cov + Link | 1.808882 | 1.640802 | 0.138750 |
| 1 | 2 | Cov + Link + Exist, shared weights | 1.785493 | 1.577481 | 0.138750 |
| 1 | 2 | Cov + Link + Exist, branch-decoupled | 1.802378 | 1.577661 | 0.140000 |
| 1 | 2 | + structure-aware | 1.776217 | 1.538749 | 0.131250 |
| 1 | 2 | + EMA/floor | 1.860476 | 1.656601 | 0.285000 |
| 1 | 2 | + FID-FIA existence only | 1.784257 | 1.582221 | 0.232500 |
| 2 | 3 | Fixed Metropolis | 2.288080 | 1.830852 | 0.597500 |
| 2 | 3 | Cov only | 1.763208 | 1.433480 | 0.281250 |
| 2 | 3 | Link only | 1.709746 | 1.387159 | 0.148750 |
| 2 | 3 | Exist only | 1.818433 | 1.562761 | 0.275000 |
| 2 | 3 | Cov + Link | 1.706633 | 1.396746 | 0.131250 |
| 2 | 3 | Cov + Link + Exist, shared weights | 1.712969 | 1.395240 | 0.135000 |
| 2 | 3 | Cov + Link + Exist, branch-decoupled | 1.711684 | 1.394570 | 0.136250 |
| 2 | 3 | + structure-aware | 1.695417 | 1.376979 | 0.142500 |
| 2 | 3 | + EMA/floor | 1.685527 | 1.319531 | 0.178750 |
| 2 | 3 | + FID-FIA existence only | 1.699866 | 1.371209 | 0.172500 |
| 3 | 4 | Fixed Metropolis | 2.437838 | 1.938555 | 0.568750 |
| 3 | 4 | Cov only | 1.969597 | 1.549477 | 0.297500 |
| 3 | 4 | Link only | 1.685457 | 1.454772 | 0.092500 |
| 3 | 4 | Exist only | 2.166075 | 1.604961 | 0.387500 |
| 3 | 4 | Cov + Link | 1.686293 | 1.461254 | 0.087500 |
| 3 | 4 | Cov + Link + Exist, shared weights | 1.686460 | 1.461569 | 0.087500 |
| 3 | 4 | Cov + Link + Exist, branch-decoupled | 1.686369 | 1.461398 | 0.087500 |
| 3 | 4 | + structure-aware | 1.668904 | 1.445966 | 0.087500 |
| 3 | 4 | + EMA/floor | 1.752068 | 1.385895 | 0.171250 |
| 3 | 4 | + FID-FIA existence only | 1.739024 | 1.371033 | 0.141250 |
| 4 | 5 | Fixed Metropolis | 2.355063 | 2.441315 | 0.440000 |
| 4 | 5 | Cov only | 2.022684 | 1.925531 | 0.310000 |
| 4 | 5 | Link only | 1.695122 | 1.490191 | 0.073750 |
| 4 | 5 | Exist only | 2.034173 | 1.650509 | 0.337500 |
| 4 | 5 | Cov + Link | 1.680418 | 1.494838 | 0.056250 |
| 4 | 5 | Cov + Link + Exist, shared weights | 1.681793 | 1.496057 | 0.056250 |
| 4 | 5 | Cov + Link + Exist, branch-decoupled | 1.681073 | 1.495419 | 0.056250 |
| 4 | 5 | + structure-aware | 1.679364 | 1.485714 | 0.065000 |
| 4 | 5 | + EMA/floor | 1.793495 | 1.610283 | 0.141250 |
| 4 | 5 | + FID-FIA existence only | 1.719090 | 1.492124 | 0.100000 |
| 5 | 6 | Fixed Metropolis | 2.570926 | 1.946042 | 0.893750 |
| 5 | 6 | Cov only | 2.067414 | 1.956966 | 0.393750 |
| 5 | 6 | Link only | 1.757188 | 1.482262 | 0.135000 |
| 5 | 6 | Exist only | 2.139595 | 1.998114 | 0.448750 |
| 5 | 6 | Cov + Link | 1.753094 | 1.493754 | 0.113750 |
| 5 | 6 | Cov + Link + Exist, shared weights | 1.757720 | 1.493594 | 0.116250 |
| 5 | 6 | Cov + Link + Exist, branch-decoupled | 1.758560 | 1.493609 | 0.117500 |
| 5 | 6 | + structure-aware | 1.743825 | 1.479810 | 0.121250 |
| 5 | 6 | + EMA/floor | 1.841902 | 1.614896 | 0.203750 |
| 5 | 6 | + FID-FIA existence only | 1.815804 | 1.428249 | 0.186250 |
| 6 | 7 | Fixed Metropolis | 2.572672 | 2.382675 | 0.713750 |
| 6 | 7 | Cov only | 2.042240 | 2.072680 | 0.366250 |
| 6 | 7 | Link only | 1.748932 | 1.487421 | 0.121250 |
| 6 | 7 | Exist only | 2.076434 | 1.815262 | 0.397500 |
| 6 | 7 | Cov + Link | 1.719647 | 1.484680 | 0.093750 |
| 6 | 7 | Cov + Link + Exist, shared weights | 1.720487 | 1.480378 | 0.093750 |
| 6 | 7 | Cov + Link + Exist, branch-decoupled | 1.720889 | 1.479463 | 0.095000 |
| 6 | 7 | + structure-aware | 1.697624 | 1.464220 | 0.093750 |
| 6 | 7 | + EMA/floor | 1.792545 | 1.658296 | 0.175000 |
| 6 | 7 | + FID-FIA existence only | 1.728790 | 1.523915 | 0.130000 |
| 7 | 8 | Fixed Metropolis | 2.017715 | 1.983872 | 0.351250 |
| 7 | 8 | Cov only | 1.773559 | 1.854951 | 0.233750 |
| 7 | 8 | Link only | 1.656779 | 1.412302 | 0.098750 |
| 7 | 8 | Exist only | 1.824721 | 1.924695 | 0.261250 |
| 7 | 8 | Cov + Link | 1.646203 | 1.415960 | 0.081250 |
| 7 | 8 | Cov + Link + Exist, shared weights | 1.640696 | 1.396610 | 0.083750 |
| 7 | 8 | Cov + Link + Exist, branch-decoupled | 1.639617 | 1.396504 | 0.083750 |
| 7 | 8 | + structure-aware | 1.621098 | 1.379179 | 0.081250 |
| 7 | 8 | + EMA/floor | 1.624252 | 1.662182 | 0.137500 |
| 7 | 8 | + FID-FIA existence only | 1.597828 | 1.423196 | 0.116250 |
| 8 | 9 | Fixed Metropolis | 2.226628 | 2.475786 | 0.492500 |
| 8 | 9 | Cov only | 1.977775 | 1.906447 | 0.331250 |
| 8 | 9 | Link only | 1.700565 | 1.424636 | 0.128750 |
| 8 | 9 | Exist only | 2.054526 | 1.735584 | 0.425000 |
| 8 | 9 | Cov + Link | 1.693305 | 1.427734 | 0.113750 |
| 8 | 9 | Cov + Link + Exist, shared weights | 1.694011 | 1.427295 | 0.112500 |
| 8 | 9 | Cov + Link + Exist, branch-decoupled | 1.695682 | 1.427599 | 0.116250 |
| 8 | 9 | + structure-aware | 1.683326 | 1.417222 | 0.116250 |
| 8 | 9 | + EMA/floor | 1.744146 | 1.704863 | 0.145000 |
| 8 | 9 | + FID-FIA existence only | 1.725909 | 1.522302 | 0.141250 |
| 9 | 10 | Fixed Metropolis | 2.284945 | 2.464929 | 0.585000 |
| 9 | 10 | Cov only | 1.913377 | 1.813299 | 0.337500 |
| 9 | 10 | Link only | 1.696331 | 1.455294 | 0.091250 |
| 9 | 10 | Exist only | 1.979124 | 1.780737 | 0.396250 |
| 9 | 10 | Cov + Link | 1.697548 | 1.465954 | 0.080000 |
| 9 | 10 | Cov + Link + Exist, shared weights | 1.702912 | 1.467422 | 0.082500 |
| 9 | 10 | Cov + Link + Exist, branch-decoupled | 1.701314 | 1.465952 | 0.082500 |
| 9 | 10 | + structure-aware | 1.689660 | 1.455397 | 0.083750 |
| 9 | 10 | + EMA/floor | 1.729683 | 1.442146 | 0.157500 |
| 9 | 10 | + FID-FIA existence only | 1.730590 | 1.567808 | 0.126250 |
| 10 | 11 | Fixed Metropolis | 2.281465 | 2.306991 | 0.482500 |
| 10 | 11 | Cov only | 2.017530 | 1.688103 | 0.373750 |
| 10 | 11 | Link only | 1.694117 | 1.463960 | 0.072500 |
| 10 | 11 | Exist only | 1.955792 | 1.670949 | 0.377500 |
| 10 | 11 | Cov + Link | 1.691849 | 1.465646 | 0.067500 |
| 10 | 11 | Cov + Link + Exist, shared weights | 1.694618 | 1.465658 | 0.067500 |
| 10 | 11 | Cov + Link + Exist, branch-decoupled | 1.694014 | 1.465197 | 0.067500 |
| 10 | 11 | + structure-aware | 1.678264 | 1.449546 | 0.067500 |
| 10 | 11 | + EMA/floor | 1.812327 | 1.501155 | 0.192500 |
| 10 | 11 | + FID-FIA existence only | 1.795714 | 1.454115 | 0.158750 |
| 11 | 12 | Fixed Metropolis | 2.177115 | 2.331028 | 0.475000 |
| 11 | 12 | Cov only | 1.891011 | 1.719020 | 0.355000 |
| 11 | 12 | Link only | 1.623504 | 1.482680 | 0.076250 |
| 11 | 12 | Exist only | 1.993960 | 1.906179 | 0.422500 |
| 11 | 12 | Cov + Link | 1.613289 | 1.487033 | 0.063750 |
| 11 | 12 | Cov + Link + Exist, shared weights | 1.623986 | 1.487398 | 0.068750 |
| 11 | 12 | Cov + Link + Exist, branch-decoupled | 1.623252 | 1.424683 | 0.070000 |
| 11 | 12 | + structure-aware | 1.622518 | 1.429309 | 0.073750 |
| 11 | 12 | + EMA/floor | 1.635028 | 1.413051 | 0.140000 |
| 11 | 12 | + FID-FIA existence only | 1.712190 | 1.389215 | 0.131250 |
| 12 | 13 | Fixed Metropolis | 2.802667 | 2.185945 | 1.490000 |
| 12 | 13 | Cov only | 2.250700 | 1.969668 | 0.830000 |
| 12 | 13 | Link only | 1.757394 | 1.484791 | 0.093750 |
| 12 | 13 | Exist only | 2.403538 | 1.841308 | 1.065000 |
| 12 | 13 | Cov + Link | 1.759994 | 1.552441 | 0.096250 |
| 12 | 13 | Cov + Link + Exist, shared weights | 1.767430 | 1.550712 | 0.100000 |
| 12 | 13 | Cov + Link + Exist, branch-decoupled | 1.758838 | 1.548353 | 0.093750 |
| 12 | 13 | + structure-aware | 1.749735 | 1.538975 | 0.093750 |
| 12 | 13 | + EMA/floor | 1.929541 | 1.800254 | 0.293750 |
| 12 | 13 | + FID-FIA existence only | 1.942005 | 1.615793 | 0.271250 |
| 13 | 14 | Fixed Metropolis | 2.395465 | 2.097958 | 0.435000 |
| 13 | 14 | Cov only | 1.915088 | 1.702316 | 0.248750 |
| 13 | 14 | Link only | 1.737539 | 1.516209 | 0.100000 |
| 13 | 14 | Exist only | 2.021445 | 1.613840 | 0.315000 |
| 13 | 14 | Cov + Link | 1.737131 | 1.471366 | 0.098750 |
| 13 | 14 | Cov + Link + Exist, shared weights | 1.740090 | 1.467710 | 0.101250 |
| 13 | 14 | Cov + Link + Exist, branch-decoupled | 1.745263 | 1.467709 | 0.107500 |
| 13 | 14 | + structure-aware | 1.710129 | 1.458339 | 0.088750 |
| 13 | 14 | + EMA/floor | 1.717199 | 1.487381 | 0.146250 |
| 13 | 14 | + FID-FIA existence only | 1.689779 | 1.361355 | 0.122500 |
| 14 | 15 | Fixed Metropolis | 2.625276 | 2.074508 | 1.192500 |
| 14 | 15 | Cov only | 2.165912 | 1.913072 | 0.747500 |
| 14 | 15 | Link only | 1.734139 | 1.583696 | 0.090000 |
| 14 | 15 | Exist only | 2.243600 | 1.932447 | 0.885000 |
| 14 | 15 | Cov + Link | 1.741417 | 1.615327 | 0.086250 |
| 14 | 15 | Cov + Link + Exist, shared weights | 1.740195 | 1.613091 | 0.085000 |
| 14 | 15 | Cov + Link + Exist, branch-decoupled | 1.744319 | 1.612345 | 0.087500 |
| 14 | 15 | + structure-aware | 1.730400 | 1.560319 | 0.088750 |
| 14 | 15 | + EMA/floor | 1.843445 | 1.578274 | 0.251250 |
| 14 | 15 | + FID-FIA existence only | 1.826532 | 1.570785 | 0.235000 |
| 15 | 16 | Fixed Metropolis | 2.361736 | 2.294322 | 0.612500 |
| 15 | 16 | Cov only | 1.975749 | 1.613987 | 0.411250 |
| 15 | 16 | Link only | 1.655218 | 1.440479 | 0.053750 |
| 15 | 16 | Exist only | 2.033229 | 1.536971 | 0.472500 |
| 15 | 16 | Cov + Link | 1.654243 | 1.445734 | 0.046250 |
| 15 | 16 | Cov + Link + Exist, shared weights | 1.660834 | 1.447768 | 0.048750 |
| 15 | 16 | Cov + Link + Exist, branch-decoupled | 1.660593 | 1.447597 | 0.048750 |
| 15 | 16 | + structure-aware | 1.646995 | 1.427477 | 0.051250 |
| 15 | 16 | + EMA/floor | 1.752657 | 1.500242 | 0.158750 |
| 15 | 16 | + FID-FIA existence only | 1.735163 | 1.529023 | 0.123750 |
| 16 | 17 | Fixed Metropolis | 2.640159 | 2.224710 | 0.941250 |
| 16 | 17 | Cov only | 2.102486 | 2.472620 | 0.433750 |
| 16 | 17 | Link only | 1.706989 | 1.551152 | 0.106250 |
| 16 | 17 | Exist only | 2.151145 | 2.227198 | 0.545000 |
| 16 | 17 | Cov + Link | 1.706410 | 1.548041 | 0.085000 |
| 16 | 17 | Cov + Link + Exist, shared weights | 1.710119 | 1.552331 | 0.083750 |
| 16 | 17 | Cov + Link + Exist, branch-decoupled | 1.708913 | 1.549738 | 0.087500 |
| 16 | 17 | + structure-aware | 1.696159 | 1.535545 | 0.093750 |
| 16 | 17 | + EMA/floor | 1.707260 | 1.445552 | 0.168750 |
| 16 | 17 | + FID-FIA existence only | 1.675977 | 1.390003 | 0.125000 |
| 17 | 18 | Fixed Metropolis | 2.394283 | 1.897528 | 0.642500 |
| 17 | 18 | Cov only | 2.040400 | 1.730593 | 0.408750 |
| 17 | 18 | Link only | 1.681864 | 1.460145 | 0.107500 |
| 17 | 18 | Exist only | 2.038334 | 1.535049 | 0.401250 |
| 17 | 18 | Cov + Link | 1.699385 | 1.475137 | 0.110000 |
| 17 | 18 | Cov + Link + Exist, shared weights | 1.700283 | 1.476105 | 0.111250 |
| 17 | 18 | Cov + Link + Exist, branch-decoupled | 1.699892 | 1.475692 | 0.111250 |
| 17 | 18 | + structure-aware | 1.684325 | 1.462125 | 0.110000 |
| 17 | 18 | + EMA/floor | 1.728008 | 1.426142 | 0.168750 |
| 17 | 18 | + FID-FIA existence only | 1.755918 | 1.415112 | 0.168750 |
| 18 | 19 | Fixed Metropolis | 2.229845 | 1.892071 | 0.577500 |
| 18 | 19 | Cov only | 1.826836 | 1.805395 | 0.273750 |
| 18 | 19 | Link only | 1.647740 | 1.398526 | 0.092500 |
| 18 | 19 | Exist only | 1.898778 | 2.111717 | 0.326250 |
| 18 | 19 | Cov + Link | 1.626339 | 1.375034 | 0.077500 |
| 18 | 19 | Cov + Link + Exist, shared weights | 1.636717 | 1.375818 | 0.082500 |
| 18 | 19 | Cov + Link + Exist, branch-decoupled | 1.636879 | 1.376071 | 0.082500 |
| 18 | 19 | + structure-aware | 1.615129 | 1.366812 | 0.083750 |
| 18 | 19 | + EMA/floor | 1.683630 | 1.477466 | 0.176250 |
| 18 | 19 | + FID-FIA existence only | 1.673751 | 1.374238 | 0.161250 |
| 19 | 20 | Fixed Metropolis | 2.442480 | 1.844791 | 0.733750 |
| 19 | 20 | Cov only | 1.978450 | 1.959527 | 0.365000 |
| 19 | 20 | Link only | 1.709060 | 1.553368 | 0.060000 |
| 19 | 20 | Exist only | 2.052858 | 2.028038 | 0.423750 |
| 19 | 20 | Cov + Link | 1.702867 | 1.562462 | 0.051250 |
| 19 | 20 | Cov + Link + Exist, shared weights | 1.704162 | 1.561938 | 0.052500 |
| 19 | 20 | Cov + Link + Exist, branch-decoupled | 1.703204 | 1.560894 | 0.052500 |
| 19 | 20 | + structure-aware | 1.690820 | 1.551347 | 0.051250 |
| 19 | 20 | + EMA/floor | 1.753040 | 1.490625 | 0.131250 |
| 19 | 20 | + FID-FIA existence only | 1.747756 | 1.441897 | 0.113750 |
| 20 | 21 | Fixed Metropolis | 2.385856 | 2.100941 | 0.815000 |
| 20 | 21 | Cov only | 2.083418 | 2.275449 | 0.485000 |
| 20 | 21 | Link only | 1.710684 | 1.428243 | 0.122500 |
| 20 | 21 | Exist only | 2.079540 | 2.078571 | 0.575000 |
| 20 | 21 | Cov + Link | 1.716966 | 1.452466 | 0.115000 |
| 20 | 21 | Cov + Link + Exist, shared weights | 1.710416 | 1.457555 | 0.116250 |
| 20 | 21 | Cov + Link + Exist, branch-decoupled | 1.709247 | 1.454948 | 0.115000 |
| 20 | 21 | + structure-aware | 1.701454 | 1.429417 | 0.116250 |
| 20 | 21 | + EMA/floor | 1.789112 | 1.808273 | 0.203750 |
| 20 | 21 | + FID-FIA existence only | 1.686332 | 1.401547 | 0.150000 |
| 21 | 22 | Fixed Metropolis | 2.472052 | 2.388735 | 0.656250 |
| 21 | 22 | Cov only | 2.177823 | 2.126053 | 0.418750 |
| 21 | 22 | Link only | 1.728251 | 1.491159 | 0.096250 |
| 21 | 22 | Exist only | 2.267600 | 1.825071 | 0.512500 |
| 21 | 22 | Cov + Link | 1.718031 | 1.448032 | 0.083750 |
| 21 | 22 | Cov + Link + Exist, shared weights | 1.726350 | 1.462008 | 0.087500 |
| 21 | 22 | Cov + Link + Exist, branch-decoupled | 1.730063 | 1.461891 | 0.088750 |
| 21 | 22 | + structure-aware | 1.715532 | 1.451002 | 0.088750 |
| 21 | 22 | + EMA/floor | 1.768065 | 1.568987 | 0.173750 |
| 21 | 22 | + FID-FIA existence only | 1.802287 | 1.673152 | 0.180000 |
| 22 | 23 | Fixed Metropolis | 2.581426 | 2.484052 | 0.766250 |
| 22 | 23 | Cov only | 2.136675 | 2.153344 | 0.446250 |
| 22 | 23 | Link only | 1.801557 | 1.560695 | 0.102500 |
| 22 | 23 | Exist only | 2.204428 | 2.042203 | 0.505000 |
| 22 | 23 | Cov + Link | 1.783653 | 1.560435 | 0.073750 |
| 22 | 23 | Cov + Link + Exist, shared weights | 1.784311 | 1.560219 | 0.073750 |
| 22 | 23 | Cov + Link + Exist, branch-decoupled | 1.784318 | 1.560015 | 0.075000 |
| 22 | 23 | + structure-aware | 1.780849 | 1.552066 | 0.076250 |
| 22 | 23 | + EMA/floor | 1.923955 | 1.612353 | 0.218750 |
| 22 | 23 | + FID-FIA existence only | 1.926014 | 1.599260 | 0.183750 |
| 23 | 24 | Fixed Metropolis | 2.423029 | 1.684850 | 0.657500 |
| 23 | 24 | Cov only | 2.030175 | 1.647579 | 0.381250 |
| 23 | 24 | Link only | 1.654109 | 1.447762 | 0.056250 |
| 23 | 24 | Exist only | 2.086143 | 1.687366 | 0.422500 |
| 23 | 24 | Cov + Link | 1.653396 | 1.451482 | 0.047500 |
| 23 | 24 | Cov + Link + Exist, shared weights | 1.661116 | 1.452925 | 0.050000 |
| 23 | 24 | Cov + Link + Exist, branch-decoupled | 1.660410 | 1.452423 | 0.050000 |
| 23 | 24 | + structure-aware | 1.659146 | 1.440621 | 0.055000 |
| 23 | 24 | + EMA/floor | 1.692696 | 1.376302 | 0.150000 |
| 23 | 24 | + FID-FIA existence only | 1.636989 | 1.342399 | 0.091250 |
| 24 | 25 | Fixed Metropolis | 2.415883 | 2.179560 | 0.663750 |
| 24 | 25 | Cov only | 1.905426 | 1.574829 | 0.323750 |
| 24 | 25 | Link only | 1.729511 | 1.446946 | 0.150000 |
| 24 | 25 | Exist only | 2.032576 | 1.521468 | 0.393750 |
| 24 | 25 | Cov + Link | 1.722821 | 1.449594 | 0.138750 |
| 24 | 25 | Cov + Link + Exist, shared weights | 1.728163 | 1.448726 | 0.138750 |
| 24 | 25 | Cov + Link + Exist, branch-decoupled | 1.727624 | 1.449036 | 0.141250 |
| 24 | 25 | + structure-aware | 1.712950 | 1.432710 | 0.137500 |
| 24 | 25 | + EMA/floor | 1.802510 | 1.568805 | 0.235000 |
| 24 | 25 | + FID-FIA existence only | 1.772789 | 1.486859 | 0.215000 |
| 25 | 26 | Fixed Metropolis | 2.339908 | 2.228652 | 0.641250 |
| 25 | 26 | Cov only | 1.938748 | 1.820054 | 0.357500 |
| 25 | 26 | Link only | 1.747953 | 1.497328 | 0.122500 |
| 25 | 26 | Exist only | 2.005825 | 1.775542 | 0.467500 |
| 25 | 26 | Cov + Link | 1.731513 | 1.483955 | 0.106250 |
| 25 | 26 | Cov + Link + Exist, shared weights | 1.735514 | 1.481639 | 0.110000 |
| 25 | 26 | Cov + Link + Exist, branch-decoupled | 1.743628 | 1.492576 | 0.112500 |
| 25 | 26 | + structure-aware | 1.710915 | 1.468458 | 0.112500 |
| 25 | 26 | + EMA/floor | 1.746858 | 1.637186 | 0.168750 |
| 25 | 26 | + FID-FIA existence only | 1.685441 | 1.527504 | 0.142500 |
| 26 | 27 | Fixed Metropolis | 2.375397 | 2.134500 | 0.803750 |
| 26 | 27 | Cov only | 1.978979 | 2.033330 | 0.388750 |
| 26 | 27 | Link only | 1.685717 | 1.369619 | 0.133750 |
| 26 | 27 | Exist only | 2.020559 | 1.887968 | 0.480000 |
| 26 | 27 | Cov + Link | 1.690582 | 1.390396 | 0.123750 |
| 26 | 27 | Cov + Link + Exist, shared weights | 1.694638 | 1.384383 | 0.123750 |
| 26 | 27 | Cov + Link + Exist, branch-decoupled | 1.697299 | 1.383735 | 0.126250 |
| 26 | 27 | + structure-aware | 1.680122 | 1.365020 | 0.127500 |
| 26 | 27 | + EMA/floor | 1.702484 | 1.394454 | 0.198750 |
| 26 | 27 | + FID-FIA existence only | 1.736079 | 1.379115 | 0.206250 |
| 27 | 28 | Fixed Metropolis | 2.256616 | 2.450563 | 0.435000 |
| 27 | 28 | Cov only | 2.000659 | 1.686629 | 0.331250 |
| 27 | 28 | Link only | 1.813934 | 1.526883 | 0.151250 |
| 27 | 28 | Exist only | 2.081604 | 1.764946 | 0.405000 |
| 27 | 28 | Cov + Link | 1.805659 | 1.529449 | 0.138750 |
| 27 | 28 | Cov + Link + Exist, shared weights | 1.816693 | 1.531016 | 0.142500 |
| 27 | 28 | Cov + Link + Exist, branch-decoupled | 1.818333 | 1.533163 | 0.141250 |
| 27 | 28 | + structure-aware | 1.806579 | 1.519863 | 0.142500 |
| 27 | 28 | + EMA/floor | 1.823602 | 1.413787 | 0.217500 |
| 27 | 28 | + FID-FIA existence only | 1.804156 | 1.441104 | 0.182500 |
| 28 | 29 | Fixed Metropolis | 2.099246 | 1.874292 | 0.376250 |
| 28 | 29 | Cov only | 1.720964 | 1.554401 | 0.240000 |
| 28 | 29 | Link only | 1.618121 | 1.344683 | 0.126250 |
| 28 | 29 | Exist only | 1.756444 | 1.514312 | 0.240000 |
| 28 | 29 | Cov + Link | 1.615647 | 1.345893 | 0.118750 |
| 28 | 29 | Cov + Link + Exist, shared weights | 1.634791 | 1.346943 | 0.123750 |
| 28 | 29 | Cov + Link + Exist, branch-decoupled | 1.629817 | 1.347516 | 0.122500 |
| 28 | 29 | + structure-aware | 1.629361 | 1.333258 | 0.126250 |
| 28 | 29 | + EMA/floor | 1.627447 | 1.284065 | 0.210000 |
| 28 | 29 | + FID-FIA existence only | 1.651819 | 1.302799 | 0.196250 |
| 29 | 30 | Fixed Metropolis | 2.313788 | 3.195284 | 0.437500 |
| 29 | 30 | Cov only | 1.904710 | 2.171243 | 0.305000 |
| 29 | 30 | Link only | 1.680048 | 1.475007 | 0.072500 |
| 29 | 30 | Exist only | 2.051825 | 2.083884 | 0.435000 |
| 29 | 30 | Cov + Link | 1.674623 | 1.463886 | 0.066250 |
| 29 | 30 | Cov + Link + Exist, shared weights | 1.676525 | 1.463800 | 0.067500 |
| 29 | 30 | Cov + Link + Exist, branch-decoupled | 1.673937 | 1.462763 | 0.067500 |
| 29 | 30 | + structure-aware | 1.655319 | 1.444053 | 0.067500 |
| 29 | 30 | + EMA/floor | 1.691656 | 1.439356 | 0.130000 |
| 29 | 30 | + FID-FIA existence only | 1.710293 | 1.477726 | 0.112500 |
| 30 | 31 | Fixed Metropolis | 2.276694 | 1.882527 | 0.650000 |
| 30 | 31 | Cov only | 1.916255 | 1.601480 | 0.367500 |
| 30 | 31 | Link only | 1.683920 | 1.426539 | 0.083750 |
| 30 | 31 | Exist only | 2.020147 | 1.673632 | 0.442500 |
| 30 | 31 | Cov + Link | 1.678187 | 1.430555 | 0.078750 |
| 30 | 31 | Cov + Link + Exist, shared weights | 1.681234 | 1.431846 | 0.080000 |
| 30 | 31 | Cov + Link + Exist, branch-decoupled | 1.683809 | 1.430488 | 0.081250 |
| 30 | 31 | + structure-aware | 1.660956 | 1.414662 | 0.078750 |
| 30 | 31 | + EMA/floor | 1.639166 | 1.379166 | 0.147500 |
| 30 | 31 | + FID-FIA existence only | 1.613867 | 1.345170 | 0.121250 |
| 31 | 32 | Fixed Metropolis | 2.471433 | 2.380578 | 0.643750 |
| 31 | 32 | Cov only | 2.090516 | 1.990204 | 0.405000 |
| 31 | 32 | Link only | 1.755967 | 1.552305 | 0.091250 |
| 31 | 32 | Exist only | 2.155431 | 1.894504 | 0.507500 |
| 31 | 32 | Cov + Link | 1.758435 | 1.570078 | 0.083750 |
| 31 | 32 | Cov + Link + Exist, shared weights | 1.764079 | 1.564939 | 0.086250 |
| 31 | 32 | Cov + Link + Exist, branch-decoupled | 1.767788 | 1.572315 | 0.090000 |
| 31 | 32 | + structure-aware | 1.755435 | 1.565038 | 0.088750 |
| 31 | 32 | + EMA/floor | 1.863285 | 1.759296 | 0.162500 |
| 31 | 32 | + FID-FIA existence only | 1.874224 | 1.690247 | 0.166250 |
| 32 | 33 | Fixed Metropolis | 2.366485 | 2.047092 | 0.668750 |
| 32 | 33 | Cov only | 1.891871 | 1.969916 | 0.296250 |
| 32 | 33 | Link only | 1.778502 | 1.512858 | 0.126250 |
| 32 | 33 | Exist only | 1.971161 | 1.890291 | 0.402500 |
| 32 | 33 | Cov + Link | 1.754774 | 1.519335 | 0.103750 |
| 32 | 33 | Cov + Link + Exist, shared weights | 1.751305 | 1.522043 | 0.100000 |
| 32 | 33 | Cov + Link + Exist, branch-decoupled | 1.751913 | 1.519675 | 0.101250 |
| 32 | 33 | + structure-aware | 1.733138 | 1.510294 | 0.095000 |
| 32 | 33 | + EMA/floor | 1.757737 | 1.597830 | 0.172500 |
| 32 | 33 | + FID-FIA existence only | 1.802133 | 1.600096 | 0.178750 |
| 33 | 34 | Fixed Metropolis | 2.271483 | 2.415170 | 0.393750 |
| 33 | 34 | Cov only | 1.861291 | 1.556753 | 0.290000 |
| 33 | 34 | Link only | 1.694604 | 1.476612 | 0.083750 |
| 33 | 34 | Exist only | 1.891093 | 1.636886 | 0.333750 |
| 33 | 34 | Cov + Link | 1.689726 | 1.483097 | 0.073750 |
| 33 | 34 | Cov + Link + Exist, shared weights | 1.690019 | 1.484870 | 0.072500 |
| 33 | 34 | Cov + Link + Exist, branch-decoupled | 1.691424 | 1.483408 | 0.076250 |
| 33 | 34 | + structure-aware | 1.677864 | 1.470743 | 0.077500 |
| 33 | 34 | + EMA/floor | 1.766659 | 1.438420 | 0.165000 |
| 33 | 34 | + FID-FIA existence only | 1.715429 | 1.400226 | 0.120000 |
| 34 | 35 | Fixed Metropolis | 2.718577 | 2.453354 | 0.921250 |
| 34 | 35 | Cov only | 2.265176 | 2.221173 | 0.511250 |
| 34 | 35 | Link only | 1.770335 | 1.463527 | 0.130000 |
| 34 | 35 | Exist only | 2.386742 | 2.159077 | 0.622500 |
| 34 | 35 | Cov + Link | 1.751585 | 1.468413 | 0.117500 |
| 34 | 35 | Cov + Link + Exist, shared weights | 1.754075 | 1.466784 | 0.118750 |
| 34 | 35 | Cov + Link + Exist, branch-decoupled | 1.755338 | 1.467578 | 0.120000 |
| 34 | 35 | + structure-aware | 1.738127 | 1.449747 | 0.121250 |
| 34 | 35 | + EMA/floor | 1.855644 | 1.412017 | 0.305000 |
| 34 | 35 | + FID-FIA existence only | 1.932411 | 1.476903 | 0.281250 |
| 35 | 36 | Fixed Metropolis | 2.353592 | 3.177383 | 0.597500 |
| 35 | 36 | Cov only | 1.973672 | 2.181597 | 0.405000 |
| 35 | 36 | Link only | 1.695810 | 1.461693 | 0.071250 |
| 35 | 36 | Exist only | 2.034831 | 2.264934 | 0.487500 |
| 35 | 36 | Cov + Link | 1.704104 | 1.513743 | 0.071250 |
| 35 | 36 | Cov + Link + Exist, shared weights | 1.709812 | 1.515226 | 0.073750 |
| 35 | 36 | Cov + Link + Exist, branch-decoupled | 1.709455 | 1.509270 | 0.073750 |
| 35 | 36 | + structure-aware | 1.692634 | 1.503576 | 0.072500 |
| 35 | 36 | + EMA/floor | 1.742447 | 1.438734 | 0.163750 |
| 35 | 36 | + FID-FIA existence only | 1.693192 | 1.396568 | 0.143750 |
| 36 | 37 | Fixed Metropolis | 2.139151 | 2.675510 | 0.307500 |
| 36 | 37 | Cov only | 1.911909 | 1.910310 | 0.245000 |
| 36 | 37 | Link only | 1.756877 | 1.416394 | 0.131250 |
| 36 | 37 | Exist only | 1.944320 | 1.817045 | 0.313750 |
| 36 | 37 | Cov + Link | 1.744614 | 1.460197 | 0.107500 |
| 36 | 37 | Cov + Link + Exist, shared weights | 1.737636 | 1.455430 | 0.108750 |
| 36 | 37 | Cov + Link + Exist, branch-decoupled | 1.742204 | 1.455277 | 0.113750 |
| 36 | 37 | + structure-aware | 1.732129 | 1.449521 | 0.110000 |
| 36 | 37 | + EMA/floor | 1.773968 | 1.663995 | 0.147500 |
| 36 | 37 | + FID-FIA existence only | 1.714421 | 1.369075 | 0.132500 |
| 37 | 38 | Fixed Metropolis | 2.285197 | 2.242812 | 0.597500 |
| 37 | 38 | Cov only | 1.858659 | 1.636528 | 0.263750 |
| 37 | 38 | Link only | 1.715531 | 1.487322 | 0.072500 |
| 37 | 38 | Exist only | 1.872913 | 1.615209 | 0.285000 |
| 37 | 38 | Cov + Link | 1.714668 | 1.484818 | 0.070000 |
| 37 | 38 | Cov + Link + Exist, shared weights | 1.712406 | 1.487440 | 0.067500 |
| 37 | 38 | Cov + Link + Exist, branch-decoupled | 1.713582 | 1.481559 | 0.066250 |
| 37 | 38 | + structure-aware | 1.699321 | 1.470785 | 0.070000 |
| 37 | 38 | + EMA/floor | 1.771367 | 1.486173 | 0.173750 |
| 37 | 38 | + FID-FIA existence only | 1.728471 | 1.448090 | 0.146250 |
| 38 | 39 | Fixed Metropolis | 2.498101 | 2.087230 | 0.726250 |
| 38 | 39 | Cov only | 2.017062 | 1.756281 | 0.353750 |
| 38 | 39 | Link only | 1.767584 | 1.450330 | 0.125000 |
| 38 | 39 | Exist only | 2.004138 | 1.901519 | 0.387500 |
| 38 | 39 | Cov + Link | 1.757440 | 1.454312 | 0.110000 |
| 38 | 39 | Cov + Link + Exist, shared weights | 1.769441 | 1.456009 | 0.111250 |
| 38 | 39 | Cov + Link + Exist, branch-decoupled | 1.768366 | 1.454664 | 0.113750 |
| 38 | 39 | + structure-aware | 1.759158 | 1.439682 | 0.125000 |
| 38 | 39 | + EMA/floor | 1.808144 | 1.617132 | 0.212500 |
| 38 | 39 | + FID-FIA existence only | 1.781895 | 1.523030 | 0.172500 |
| 39 | 40 | Fixed Metropolis | 2.491049 | 2.138481 | 0.721250 |
| 39 | 40 | Cov only | 2.007489 | 1.948904 | 0.366250 |
| 39 | 40 | Link only | 1.696594 | 1.464311 | 0.113750 |
| 39 | 40 | Exist only | 2.139774 | 1.963999 | 0.442500 |
| 39 | 40 | Cov + Link | 1.699277 | 1.447122 | 0.123750 |
| 39 | 40 | Cov + Link + Exist, shared weights | 1.697885 | 1.444161 | 0.120000 |
| 39 | 40 | Cov + Link + Exist, branch-decoupled | 1.696787 | 1.420356 | 0.120000 |
| 39 | 40 | + structure-aware | 1.674893 | 1.404751 | 0.117500 |
| 39 | 40 | + EMA/floor | 1.739855 | 1.566623 | 0.198750 |
| 39 | 40 | + FID-FIA existence only | 1.709634 | 1.438613 | 0.156250 |
| 40 | 41 | Fixed Metropolis | 2.467385 | 2.255943 | 0.645000 |
| 40 | 41 | Cov only | 1.929826 | 1.684609 | 0.332500 |
| 40 | 41 | Link only | 1.707480 | 1.442727 | 0.130000 |
| 40 | 41 | Exist only | 2.039836 | 1.671916 | 0.380000 |
| 40 | 41 | Cov + Link | 1.700493 | 1.450990 | 0.095000 |
| 40 | 41 | Cov + Link + Exist, shared weights | 1.700067 | 1.448410 | 0.096250 |
| 40 | 41 | Cov + Link + Exist, branch-decoupled | 1.699685 | 1.446380 | 0.098750 |
| 40 | 41 | + structure-aware | 1.682575 | 1.435045 | 0.095000 |
| 40 | 41 | + EMA/floor | 1.791553 | 1.529340 | 0.195000 |
| 40 | 41 | + FID-FIA existence only | 1.809302 | 1.533178 | 0.191250 |
| 41 | 42 | Fixed Metropolis | 2.050743 | 2.291038 | 0.293750 |
| 41 | 42 | Cov only | 1.826892 | 1.686375 | 0.266250 |
| 41 | 42 | Link only | 1.645549 | 1.455979 | 0.085000 |
| 41 | 42 | Exist only | 1.941618 | 1.803276 | 0.323750 |
| 41 | 42 | Cov + Link | 1.668786 | 1.501587 | 0.087500 |
| 41 | 42 | Cov + Link + Exist, shared weights | 1.663252 | 1.471347 | 0.086250 |
| 41 | 42 | Cov + Link + Exist, branch-decoupled | 1.661747 | 1.465098 | 0.085000 |
| 41 | 42 | + structure-aware | 1.645972 | 1.445265 | 0.085000 |
| 41 | 42 | + EMA/floor | 1.673512 | 1.366630 | 0.158750 |
| 41 | 42 | + FID-FIA existence only | 1.677446 | 1.371955 | 0.118750 |
| 42 | 43 | Fixed Metropolis | 2.305876 | 2.476702 | 0.663750 |
| 42 | 43 | Cov only | 1.984152 | 1.969946 | 0.398750 |
| 42 | 43 | Link only | 1.617656 | 1.431427 | 0.065000 |
| 42 | 43 | Exist only | 2.050165 | 2.024186 | 0.516250 |
| 42 | 43 | Cov + Link | 1.617064 | 1.434799 | 0.058750 |
| 42 | 43 | Cov + Link + Exist, shared weights | 1.619601 | 1.435931 | 0.060000 |
| 42 | 43 | Cov + Link + Exist, branch-decoupled | 1.619535 | 1.434946 | 0.063750 |
| 42 | 43 | + structure-aware | 1.597399 | 1.416860 | 0.062500 |
| 42 | 43 | + EMA/floor | 1.651937 | 1.471229 | 0.133750 |
| 42 | 43 | + FID-FIA existence only | 1.675525 | 1.424281 | 0.135000 |
| 43 | 44 | Fixed Metropolis | 2.449409 | 2.066644 | 0.815000 |
| 43 | 44 | Cov only | 2.046848 | 1.895356 | 0.487500 |
| 43 | 44 | Link only | 1.712491 | 1.485150 | 0.095000 |
| 43 | 44 | Exist only | 2.165872 | 1.880841 | 0.643750 |
| 43 | 44 | Cov + Link | 1.700332 | 1.485681 | 0.086250 |
| 43 | 44 | Cov + Link + Exist, shared weights | 1.695774 | 1.484750 | 0.083750 |
| 43 | 44 | Cov + Link + Exist, branch-decoupled | 1.694927 | 1.484024 | 0.083750 |
| 43 | 44 | + structure-aware | 1.688984 | 1.471778 | 0.086250 |
| 43 | 44 | + EMA/floor | 1.865991 | 1.625554 | 0.252500 |
| 43 | 44 | + FID-FIA existence only | 1.834616 | 1.748580 | 0.220000 |
| 44 | 45 | Fixed Metropolis | 2.625025 | 2.545403 | 0.967500 |
| 44 | 45 | Cov only | 2.131967 | 2.301745 | 0.480000 |
| 44 | 45 | Link only | 1.756961 | 1.536507 | 0.087500 |
| 44 | 45 | Exist only | 2.241203 | 2.308601 | 0.626250 |
| 44 | 45 | Cov + Link | 1.744808 | 1.518437 | 0.068750 |
| 44 | 45 | Cov + Link + Exist, shared weights | 1.744182 | 1.518906 | 0.067500 |
| 44 | 45 | Cov + Link + Exist, branch-decoupled | 1.746736 | 1.518412 | 0.070000 |
| 44 | 45 | + structure-aware | 1.733828 | 1.507640 | 0.070000 |
| 44 | 45 | + EMA/floor | 1.817003 | 1.751807 | 0.146250 |
| 44 | 45 | + FID-FIA existence only | 1.769065 | 1.568134 | 0.137500 |
| 45 | 46 | Fixed Metropolis | 2.116242 | 2.253473 | 0.345000 |
| 45 | 46 | Cov only | 1.854481 | 1.668001 | 0.262500 |
| 45 | 46 | Link only | 1.692445 | 1.484245 | 0.125000 |
| 45 | 46 | Exist only | 1.807514 | 1.686962 | 0.271250 |
| 45 | 46 | Cov + Link | 1.685334 | 1.462088 | 0.118750 |
| 45 | 46 | Cov + Link + Exist, shared weights | 1.700878 | 1.499518 | 0.130000 |
| 45 | 46 | Cov + Link + Exist, branch-decoupled | 1.698334 | 1.512113 | 0.125000 |
| 45 | 46 | + structure-aware | 1.679042 | 1.440953 | 0.128750 |
| 45 | 46 | + EMA/floor | 1.650981 | 1.402007 | 0.143750 |
| 45 | 46 | + FID-FIA existence only | 1.688221 | 1.453192 | 0.128750 |
| 46 | 47 | Fixed Metropolis | 2.557554 | 2.003232 | 0.800000 |
| 46 | 47 | Cov only | 2.011802 | 1.889212 | 0.340000 |
| 46 | 47 | Link only | 1.673097 | 1.504844 | 0.067500 |
| 46 | 47 | Exist only | 2.188724 | 1.927621 | 0.452500 |
| 46 | 47 | Cov + Link | 1.679066 | 1.513663 | 0.062500 |
| 46 | 47 | Cov + Link + Exist, shared weights | 1.680297 | 1.515054 | 0.062500 |
| 46 | 47 | Cov + Link + Exist, branch-decoupled | 1.674747 | 1.513854 | 0.062500 |
| 46 | 47 | + structure-aware | 1.667403 | 1.504311 | 0.063750 |
| 46 | 47 | + EMA/floor | 1.708203 | 1.379627 | 0.116250 |
| 46 | 47 | + FID-FIA existence only | 1.687675 | 1.421743 | 0.085000 |
| 47 | 48 | Fixed Metropolis | 2.464337 | 1.982020 | 0.650000 |
| 47 | 48 | Cov only | 2.082917 | 1.770990 | 0.382500 |
| 47 | 48 | Link only | 1.651139 | 1.431918 | 0.078750 |
| 47 | 48 | Exist only | 2.155493 | 1.801726 | 0.428750 |
| 47 | 48 | Cov + Link | 1.657949 | 1.436152 | 0.076250 |
| 47 | 48 | Cov + Link + Exist, shared weights | 1.655267 | 1.444437 | 0.073750 |
| 47 | 48 | Cov + Link + Exist, branch-decoupled | 1.656060 | 1.443453 | 0.075000 |
| 47 | 48 | + structure-aware | 1.647420 | 1.436031 | 0.080000 |
| 47 | 48 | + EMA/floor | 1.749885 | 1.534725 | 0.172500 |
| 47 | 48 | + FID-FIA existence only | 1.700287 | 1.452159 | 0.133750 |
| 48 | 49 | Fixed Metropolis | 2.549801 | 2.281266 | 0.801250 |
| 48 | 49 | Cov only | 2.055706 | 2.302926 | 0.345000 |
| 48 | 49 | Link only | 1.727414 | 1.485065 | 0.125000 |
| 48 | 49 | Exist only | 2.152446 | 1.954047 | 0.478750 |
| 48 | 49 | Cov + Link | 1.738750 | 1.515927 | 0.121250 |
| 48 | 49 | Cov + Link + Exist, shared weights | 1.742182 | 1.516019 | 0.121250 |
| 48 | 49 | Cov + Link + Exist, branch-decoupled | 1.743955 | 1.521252 | 0.125000 |
| 48 | 49 | + structure-aware | 1.714375 | 1.477313 | 0.121250 |
| 48 | 49 | + EMA/floor | 1.824226 | 1.582437 | 0.195000 |
| 48 | 49 | + FID-FIA existence only | 1.747230 | 1.435013 | 0.160000 |
| 49 | 50 | Fixed Metropolis | 2.007995 | 2.021656 | 0.353750 |
| 49 | 50 | Cov only | 1.772449 | 1.564794 | 0.256250 |
| 49 | 50 | Link only | 1.713386 | 1.445408 | 0.093750 |
| 49 | 50 | Exist only | 1.781067 | 1.571969 | 0.261250 |
| 49 | 50 | Cov + Link | 1.723088 | 1.598774 | 0.087500 |
| 49 | 50 | Cov + Link + Exist, shared weights | 1.718572 | 1.559809 | 0.083750 |
| 49 | 50 | Cov + Link + Exist, branch-decoupled | 1.716050 | 1.512051 | 0.090000 |
| 49 | 50 | + structure-aware | 1.694142 | 1.433640 | 0.087500 |
| 49 | 50 | + EMA/floor | 1.724113 | 1.432430 | 0.127500 |
| 49 | 50 | + FID-FIA existence only | 1.708804 | 1.425179 | 0.116250 |
| 50 | 51 | Fixed Metropolis | 2.567926 | 2.994290 | 0.692500 |
| 50 | 51 | Cov only | 2.018515 | 2.216674 | 0.312500 |
| 50 | 51 | Link only | 1.787920 | 1.525728 | 0.125000 |
| 50 | 51 | Exist only | 2.175711 | 2.280026 | 0.417500 |
| 50 | 51 | Cov + Link | 1.792457 | 1.530769 | 0.127500 |
| 50 | 51 | Cov + Link + Exist, shared weights | 1.794097 | 1.542991 | 0.126250 |
| 50 | 51 | Cov + Link + Exist, branch-decoupled | 1.791629 | 1.542131 | 0.123750 |
| 50 | 51 | + structure-aware | 1.780681 | 1.532844 | 0.123750 |
| 50 | 51 | + EMA/floor | 1.885907 | 2.074941 | 0.180000 |
| 50 | 51 | + FID-FIA existence only | 1.879545 | 1.977165 | 0.176250 |

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| Fixed Metropolis | 2.383288 | 2.263492 | 0.650100 |
| Cov only | 1.983268 | 1.876524 | 0.371975 |
| Link only | 1.711686 | 1.473528 | 0.102400 |
| Exist only | 2.057064 | 1.856968 | 0.447450 |
| Cov + Link | 1.707975 | 1.482721 | 0.092425 |
| Cov + Link + Exist, shared weights | 1.710231 | 1.480986 | 0.093425 |
| Cov + Link + Exist, branch-decoupled | 1.710630 | 1.478137 | 0.094450 |
| + structure-aware | 1.695752 | 1.460800 | 0.094700 |
| + EMA/floor | 1.760324 | 1.535770 | 0.179975 |
| + FID-FIA existence only | 1.745031 | 1.479073 | 0.156775 |

## Network Disagreement Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Fixed Metropolis | OSPA | 2.383288 +/- 0.176309 | [2.334418, 2.432158] | 50 |
| Cov only | OSPA | 1.983268 +/- 0.120416 | [1.949891, 2.016646] | 50 |
| Link only | OSPA | 1.711686 +/- 0.048604 | [1.698214, 1.725158] | 50 |
| Exist only | OSPA | 2.057064 +/- 0.143262 | [2.017353, 2.096774] | 50 |
| Cov + Link | OSPA | 1.707975 +/- 0.046432 | [1.695105, 1.720846] | 50 |
| Cov + Link + Exist, shared weights | OSPA | 1.710231 +/- 0.044947 | [1.697772, 1.722690] | 50 |
| Cov + Link + Exist, branch-decoupled | OSPA | 1.710630 +/- 0.046236 | [1.697814, 1.723446] | 50 |
| + structure-aware | OSPA | 1.695752 +/- 0.045525 | [1.683133, 1.708371] | 50 |
| + EMA/floor | OSPA | 1.760324 +/- 0.075878 | [1.739292, 1.781356] | 50 |
| + FID-FIA existence only | OSPA | 1.745031 +/- 0.077285 | [1.723608, 1.766453] | 50 |
| Fixed Metropolis | RMSE | 2.263492 +/- 0.340230 | [2.169185, 2.357799] | 50 |
| Cov only | RMSE | 1.876524 +/- 0.239959 | [1.810010, 1.943037] | 50 |
| Link only | RMSE | 1.473528 +/- 0.053325 | [1.458747, 1.488309] | 50 |
| Exist only | RMSE | 1.856968 +/- 0.228041 | [1.793758, 1.920177] | 50 |
| Cov + Link | RMSE | 1.482721 +/- 0.058597 | [1.466479, 1.498964] | 50 |
| Cov + Link + Exist, shared weights | RMSE | 1.480986 +/- 0.054828 | [1.465789, 1.496184] | 50 |
| Cov + Link + Exist, branch-decoupled | RMSE | 1.478137 +/- 0.055102 | [1.462863, 1.493410] | 50 |
| + structure-aware | RMSE | 1.460800 +/- 0.053302 | [1.446025, 1.475574] | 50 |
| + EMA/floor | RMSE | 1.535770 +/- 0.148402 | [1.494635, 1.576905] | 50 |
| + FID-FIA existence only | RMSE | 1.479073 +/- 0.120565 | [1.445654, 1.512492] | 50 |
| Fixed Metropolis | Cardinality | 0.650100 +/- 0.223202 | [0.588232, 0.711968] | 50 |
| Cov only | Cardinality | 0.371975 +/- 0.113812 | [0.340428, 0.403522] | 50 |
| Link only | Cardinality | 0.102400 +/- 0.026477 | [0.095061, 0.109739] | 50 |
| Exist only | Cardinality | 0.447450 +/- 0.152224 | [0.405256, 0.489644] | 50 |
| Cov + Link | Cardinality | 0.092425 +/- 0.025338 | [0.085402, 0.099448] | 50 |
| Cov + Link + Exist, shared weights | Cardinality | 0.093425 +/- 0.025664 | [0.086311, 0.100539] | 50 |
| Cov + Link + Exist, branch-decoupled | Cardinality | 0.094450 +/- 0.025655 | [0.087339, 0.101561] | 50 |
| + structure-aware | Cardinality | 0.094700 +/- 0.025104 | [0.087742, 0.101658] | 50 |
| + EMA/floor | Cardinality | 0.179975 +/- 0.042789 | [0.168114, 0.191836] | 50 |
| + FID-FIA existence only | Cardinality | 0.156775 +/- 0.043004 | [0.144855, 0.168695] | 50 |

## Paired Improvements Relative to Fixed Metropolis
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Cov only | OSPA | 0.400020 +/- 0.098937 | [0.372596, 0.427444] | 16.78% | 50/50 | 1.776e-15 |
| Link only | OSPA | 0.671602 +/- 0.159917 | [0.627275, 0.715929] | 28.18% | 50/50 | 1.776e-15 |
| Exist only | OSPA | 0.326225 +/- 0.092907 | [0.300472, 0.351977] | 13.69% | 50/50 | 1.776e-15 |
| Cov + Link | OSPA | 0.675313 +/- 0.160069 | [0.630944, 0.719682] | 28.34% | 50/50 | 1.776e-15 |
| Cov + Link + Exist, shared weights | OSPA | 0.673057 +/- 0.159687 | [0.628794, 0.717320] | 28.24% | 50/50 | 1.776e-15 |
| Cov + Link + Exist, branch-decoupled | OSPA | 0.672658 +/- 0.160139 | [0.628270, 0.717047] | 28.22% | 50/50 | 1.776e-15 |
| + structure-aware | OSPA | 0.687536 +/- 0.159399 | [0.643353, 0.731719] | 28.85% | 50/50 | 1.776e-15 |
| + EMA/floor | OSPA | 0.622964 +/- 0.137957 | [0.584725, 0.661204] | 26.14% | 50/50 | 1.776e-15 |
| + FID-FIA existence only | OSPA | 0.638257 +/- 0.141454 | [0.599048, 0.677466] | 26.78% | 50/50 | 1.776e-15 |
| Cov only | RMSE | 0.386968 +/- 0.312839 | [0.300254, 0.473683] | 17.10% | 45/50 | 4.21e-09 |
| Link only | RMSE | 0.789964 +/- 0.325590 | [0.699715, 0.880213] | 34.90% | 50/50 | 1.776e-15 |
| Exist only | RMSE | 0.406524 +/- 0.294767 | [0.324819, 0.488229] | 17.96% | 45/50 | 4.21e-09 |
| Cov + Link | RMSE | 0.780771 +/- 0.325644 | [0.690507, 0.871035] | 34.49% | 50/50 | 1.776e-15 |
| Cov + Link + Exist, shared weights | RMSE | 0.782506 +/- 0.327314 | [0.691779, 0.873233] | 34.57% | 50/50 | 1.776e-15 |
| Cov + Link + Exist, branch-decoupled | RMSE | 0.785355 +/- 0.326920 | [0.694738, 0.875973] | 34.70% | 50/50 | 1.776e-15 |
| + structure-aware | RMSE | 0.802692 +/- 0.325879 | [0.712363, 0.893022] | 35.46% | 50/50 | 1.776e-15 |
| + EMA/floor | RMSE | 0.727722 +/- 0.322780 | [0.638252, 0.817192] | 32.15% | 50/50 | 1.776e-15 |
| + FID-FIA existence only | RMSE | 0.784419 +/- 0.312960 | [0.697671, 0.871167] | 34.66% | 50/50 | 1.776e-15 |
| Cov only | Cardinality | 0.278125 +/- 0.136562 | [0.240272, 0.315978] | 42.78% | 50/50 | 1.776e-15 |
| Link only | Cardinality | 0.547700 +/- 0.223647 | [0.485708, 0.609692] | 84.25% | 50/50 | 1.776e-15 |
| Exist only | Cardinality | 0.202650 +/- 0.119268 | [0.169591, 0.235709] | 31.17% | 48/50 | 2.267e-12 |
| Cov + Link | Cardinality | 0.557675 +/- 0.223044 | [0.495850, 0.619500] | 85.78% | 50/50 | 1.776e-15 |
| Cov + Link + Exist, shared weights | Cardinality | 0.556675 +/- 0.223534 | [0.494715, 0.618635] | 85.63% | 50/50 | 1.776e-15 |
| Cov + Link + Exist, branch-decoupled | Cardinality | 0.555650 +/- 0.223916 | [0.493584, 0.617716] | 85.47% | 50/50 | 1.776e-15 |
| + structure-aware | Cardinality | 0.555400 +/- 0.223446 | [0.493464, 0.617336] | 85.43% | 50/50 | 1.776e-15 |
| + EMA/floor | Cardinality | 0.470125 +/- 0.201102 | [0.414382, 0.525868] | 72.32% | 50/50 | 1.776e-15 |
| + FID-FIA existence only | Cardinality | 0.493325 +/- 0.201843 | [0.437377, 0.549273] | 75.88% | 50/50 | 1.776e-15 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Fixed Metropolis | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Fixed Metropolis | 4.387345 +/- 1.043499 | 0.043873 | 1.000x | 50 |
| Cov only | 4.751400 +/- 1.123530 | 0.047514 | 1.084x | 50 |
| Link only | 4.718887 +/- 1.105011 | 0.047189 | 1.078x | 50 |
| Exist only | 4.866622 +/- 1.181630 | 0.048666 | 1.112x | 50 |
| Cov + Link | 4.835043 +/- 1.188044 | 0.048350 | 1.115x | 50 |
| Cov + Link + Exist, shared weights | 4.888036 +/- 1.215938 | 0.048880 | 1.123x | 50 |
| Cov + Link + Exist, branch-decoupled | 4.812558 +/- 1.176000 | 0.048126 | 1.102x | 50 |
| + structure-aware | 4.792654 +/- 1.154878 | 0.047927 | 1.099x | 50 |
| + EMA/floor | 4.794630 +/- 1.167649 | 0.047946 | 1.100x | 50 |
| + FID-FIA existence only | 11.983348 +/- 2.629673 | 0.119833 | 2.770x | 50 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| Cov only | 0.364054 +/- 0.136141 | 8.36% | 50/50 |
| Link only | 0.331542 +/- 0.263939 | 7.77% | 50/50 |
| Exist only | 0.479277 +/- 0.455555 | 11.22% | 50/50 |
| Cov + Link | 0.447698 +/- 0.867475 | 11.50% | 48/50 |
| Cov + Link + Exist, shared weights | 0.500690 +/- 0.775552 | 12.35% | 47/50 |
| Cov + Link + Exist, branch-decoupled | 0.425213 +/- 0.563400 | 10.20% | 49/50 |
| + structure-aware | 0.405308 +/- 0.604751 | 9.92% | 49/50 |
| + EMA/floor | 0.407285 +/- 0.637827 | 9.98% | 48/50 |
| + FID-FIA existence only | 7.596002 +/- 2.010425 | 176.96% | 50/50 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to Fixed Metropolis |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | Fixed Metropolis | 4.134692 | 0.041347 | 1.000x |
| 1 | 2 | Cov only | 4.403662 | 0.044037 | 1.065x |
| 1 | 2 | Link only | 4.137338 | 0.041373 | 1.001x |
| 1 | 2 | Exist only | 4.345069 | 0.043451 | 1.051x |
| 1 | 2 | Cov + Link | 4.074393 | 0.040744 | 0.985x |
| 1 | 2 | Cov + Link + Exist, shared weights | 4.115622 | 0.041156 | 0.995x |
| 1 | 2 | Cov + Link + Exist, branch-decoupled | 4.136891 | 0.041369 | 1.001x |
| 1 | 2 | + structure-aware | 4.315708 | 0.043157 | 1.044x |
| 1 | 2 | + EMA/floor | 4.205044 | 0.042050 | 1.017x |
| 1 | 2 | + FID-FIA existence only | 10.690294 | 0.106903 | 2.586x |
| 2 | 3 | Fixed Metropolis | 3.818844 | 0.038188 | 1.000x |
| 2 | 3 | Cov only | 4.199436 | 0.041994 | 1.100x |
| 2 | 3 | Link only | 4.267109 | 0.042671 | 1.117x |
| 2 | 3 | Exist only | 4.943592 | 0.049436 | 1.295x |
| 2 | 3 | Cov + Link | 6.908875 | 0.069089 | 1.809x |
| 2 | 3 | Cov + Link + Exist, shared weights | 7.029886 | 0.070299 | 1.841x |
| 2 | 3 | Cov + Link + Exist, branch-decoupled | 6.778137 | 0.067781 | 1.775x |
| 2 | 3 | + structure-aware | 6.810742 | 0.068107 | 1.783x |
| 2 | 3 | + EMA/floor | 6.809215 | 0.068092 | 1.783x |
| 2 | 3 | + FID-FIA existence only | 16.357772 | 0.163578 | 4.283x |
| 3 | 4 | Fixed Metropolis | 6.246284 | 0.062463 | 1.000x |
| 3 | 4 | Cov only | 6.816584 | 0.068166 | 1.091x |
| 3 | 4 | Link only | 6.839596 | 0.068396 | 1.095x |
| 3 | 4 | Exist only | 7.049164 | 0.070492 | 1.129x |
| 3 | 4 | Cov + Link | 6.823130 | 0.068231 | 1.092x |
| 3 | 4 | Cov + Link + Exist, shared weights | 6.853301 | 0.068533 | 1.097x |
| 3 | 4 | Cov + Link + Exist, branch-decoupled | 5.766329 | 0.057663 | 0.923x |
| 3 | 4 | + structure-aware | 4.692626 | 0.046926 | 0.751x |
| 3 | 4 | + EMA/floor | 4.197244 | 0.041972 | 0.672x |
| 3 | 4 | + FID-FIA existence only | 10.324999 | 0.103250 | 1.653x |
| 4 | 5 | Fixed Metropolis | 3.846810 | 0.038468 | 1.000x |
| 4 | 5 | Cov only | 4.315622 | 0.043156 | 1.122x |
| 4 | 5 | Link only | 4.374242 | 0.043742 | 1.137x |
| 4 | 5 | Exist only | 4.397293 | 0.043973 | 1.143x |
| 4 | 5 | Cov + Link | 4.326937 | 0.043269 | 1.125x |
| 4 | 5 | Cov + Link + Exist, shared weights | 4.294907 | 0.042949 | 1.116x |
| 4 | 5 | Cov + Link + Exist, branch-decoupled | 4.614463 | 0.046145 | 1.200x |
| 4 | 5 | + structure-aware | 4.389819 | 0.043898 | 1.141x |
| 4 | 5 | + EMA/floor | 4.204138 | 0.042041 | 1.093x |
| 4 | 5 | + FID-FIA existence only | 10.621909 | 0.106219 | 2.761x |
| 5 | 6 | Fixed Metropolis | 4.090281 | 0.040903 | 1.000x |
| 5 | 6 | Cov only | 4.246944 | 0.042469 | 1.038x |
| 5 | 6 | Link only | 4.373088 | 0.043731 | 1.069x |
| 5 | 6 | Exist only | 4.470597 | 0.044706 | 1.093x |
| 5 | 6 | Cov + Link | 4.357189 | 0.043572 | 1.065x |
| 5 | 6 | Cov + Link + Exist, shared weights | 4.256056 | 0.042561 | 1.041x |
| 5 | 6 | Cov + Link + Exist, branch-decoupled | 4.143981 | 0.041440 | 1.013x |
| 5 | 6 | + structure-aware | 4.263791 | 0.042638 | 1.042x |
| 5 | 6 | + EMA/floor | 4.309250 | 0.043093 | 1.054x |
| 5 | 6 | + FID-FIA existence only | 11.128156 | 0.111282 | 2.721x |
| 6 | 7 | Fixed Metropolis | 3.842096 | 0.038421 | 1.000x |
| 6 | 7 | Cov only | 4.137713 | 0.041377 | 1.077x |
| 6 | 7 | Link only | 4.135301 | 0.041353 | 1.076x |
| 6 | 7 | Exist only | 4.101004 | 0.041010 | 1.067x |
| 6 | 7 | Cov + Link | 4.098693 | 0.040987 | 1.067x |
| 6 | 7 | Cov + Link + Exist, shared weights | 4.257633 | 0.042576 | 1.108x |
| 6 | 7 | Cov + Link + Exist, branch-decoupled | 4.196550 | 0.041965 | 1.092x |
| 6 | 7 | + structure-aware | 4.167450 | 0.041675 | 1.085x |
| 6 | 7 | + EMA/floor | 4.102275 | 0.041023 | 1.068x |
| 6 | 7 | + FID-FIA existence only | 10.547734 | 0.105477 | 2.745x |
| 7 | 8 | Fixed Metropolis | 3.827400 | 0.038274 | 1.000x |
| 7 | 8 | Cov only | 4.268457 | 0.042685 | 1.115x |
| 7 | 8 | Link only | 4.347414 | 0.043474 | 1.136x |
| 7 | 8 | Exist only | 4.090306 | 0.040903 | 1.069x |
| 7 | 8 | Cov + Link | 4.253275 | 0.042533 | 1.111x |
| 7 | 8 | Cov + Link + Exist, shared weights | 4.275435 | 0.042754 | 1.117x |
| 7 | 8 | Cov + Link + Exist, branch-decoupled | 4.116350 | 0.041164 | 1.075x |
| 7 | 8 | + structure-aware | 4.151095 | 0.041511 | 1.085x |
| 7 | 8 | + EMA/floor | 4.149213 | 0.041492 | 1.084x |
| 7 | 8 | + FID-FIA existence only | 10.040275 | 0.100403 | 2.623x |
| 8 | 9 | Fixed Metropolis | 3.853513 | 0.038535 | 1.000x |
| 8 | 9 | Cov only | 4.241346 | 0.042413 | 1.101x |
| 8 | 9 | Link only | 4.132964 | 0.041330 | 1.073x |
| 8 | 9 | Exist only | 4.144837 | 0.041448 | 1.076x |
| 8 | 9 | Cov + Link | 4.154395 | 0.041544 | 1.078x |
| 8 | 9 | Cov + Link + Exist, shared weights | 4.186881 | 0.041869 | 1.087x |
| 8 | 9 | Cov + Link + Exist, branch-decoupled | 4.216589 | 0.042166 | 1.094x |
| 8 | 9 | + structure-aware | 4.263808 | 0.042638 | 1.106x |
| 8 | 9 | + EMA/floor | 4.398160 | 0.043982 | 1.141x |
| 8 | 9 | + FID-FIA existence only | 10.570293 | 0.105703 | 2.743x |
| 9 | 10 | Fixed Metropolis | 4.078458 | 0.040785 | 1.000x |
| 9 | 10 | Cov only | 4.120910 | 0.041209 | 1.010x |
| 9 | 10 | Link only | 4.118917 | 0.041189 | 1.010x |
| 9 | 10 | Exist only | 4.134915 | 0.041349 | 1.014x |
| 9 | 10 | Cov + Link | 4.110900 | 0.041109 | 1.008x |
| 9 | 10 | Cov + Link + Exist, shared weights | 4.062095 | 0.040621 | 0.996x |
| 9 | 10 | Cov + Link + Exist, branch-decoupled | 4.144179 | 0.041442 | 1.016x |
| 9 | 10 | + structure-aware | 4.197626 | 0.041976 | 1.029x |
| 9 | 10 | + EMA/floor | 4.074430 | 0.040744 | 0.999x |
| 9 | 10 | + FID-FIA existence only | 10.516536 | 0.105165 | 2.579x |
| 10 | 11 | Fixed Metropolis | 4.015400 | 0.040154 | 1.000x |
| 10 | 11 | Cov only | 4.385721 | 0.043857 | 1.092x |
| 10 | 11 | Link only | 4.194808 | 0.041948 | 1.045x |
| 10 | 11 | Exist only | 4.428979 | 0.044290 | 1.103x |
| 10 | 11 | Cov + Link | 4.286607 | 0.042866 | 1.068x |
| 10 | 11 | Cov + Link + Exist, shared weights | 4.185068 | 0.041851 | 1.042x |
| 10 | 11 | Cov + Link + Exist, branch-decoupled | 4.355725 | 0.043557 | 1.085x |
| 10 | 11 | + structure-aware | 4.219822 | 0.042198 | 1.051x |
| 10 | 11 | + EMA/floor | 4.191682 | 0.041917 | 1.044x |
| 10 | 11 | + FID-FIA existence only | 11.012966 | 0.110130 | 2.743x |
| 11 | 12 | Fixed Metropolis | 3.936664 | 0.039367 | 1.000x |
| 11 | 12 | Cov only | 4.157566 | 0.041576 | 1.056x |
| 11 | 12 | Link only | 4.039668 | 0.040397 | 1.026x |
| 11 | 12 | Exist only | 4.214960 | 0.042150 | 1.071x |
| 11 | 12 | Cov + Link | 4.064956 | 0.040650 | 1.033x |
| 11 | 12 | Cov + Link + Exist, shared weights | 4.089481 | 0.040895 | 1.039x |
| 11 | 12 | Cov + Link + Exist, branch-decoupled | 4.156092 | 0.041561 | 1.056x |
| 11 | 12 | + structure-aware | 4.167098 | 0.041671 | 1.059x |
| 11 | 12 | + EMA/floor | 4.057402 | 0.040574 | 1.031x |
| 11 | 12 | + FID-FIA existence only | 9.917988 | 0.099180 | 2.519x |
| 12 | 13 | Fixed Metropolis | 3.949037 | 0.039490 | 1.000x |
| 12 | 13 | Cov only | 4.223467 | 0.042235 | 1.069x |
| 12 | 13 | Link only | 4.366175 | 0.043662 | 1.106x |
| 12 | 13 | Exist only | 4.182187 | 0.041822 | 1.059x |
| 12 | 13 | Cov + Link | 4.081951 | 0.040820 | 1.034x |
| 12 | 13 | Cov + Link + Exist, shared weights | 4.173374 | 0.041734 | 1.057x |
| 12 | 13 | Cov + Link + Exist, branch-decoupled | 4.136786 | 0.041368 | 1.048x |
| 12 | 13 | + structure-aware | 4.119051 | 0.041191 | 1.043x |
| 12 | 13 | + EMA/floor | 4.197228 | 0.041972 | 1.063x |
| 12 | 13 | + FID-FIA existence only | 10.778945 | 0.107789 | 2.730x |
| 13 | 14 | Fixed Metropolis | 3.901644 | 0.039016 | 1.000x |
| 13 | 14 | Cov only | 4.202941 | 0.042029 | 1.077x |
| 13 | 14 | Link only | 4.344094 | 0.043441 | 1.113x |
| 13 | 14 | Exist only | 4.243900 | 0.042439 | 1.088x |
| 13 | 14 | Cov + Link | 4.201791 | 0.042018 | 1.077x |
| 13 | 14 | Cov + Link + Exist, shared weights | 4.143095 | 0.041431 | 1.062x |
| 13 | 14 | Cov + Link + Exist, branch-decoupled | 4.166963 | 0.041670 | 1.068x |
| 13 | 14 | + structure-aware | 4.236090 | 0.042361 | 1.086x |
| 13 | 14 | + EMA/floor | 4.309660 | 0.043097 | 1.105x |
| 13 | 14 | + FID-FIA existence only | 10.469532 | 0.104695 | 2.683x |
| 14 | 15 | Fixed Metropolis | 3.936866 | 0.039369 | 1.000x |
| 14 | 15 | Cov only | 4.252623 | 0.042526 | 1.080x |
| 14 | 15 | Link only | 4.111556 | 0.041116 | 1.044x |
| 14 | 15 | Exist only | 4.197238 | 0.041972 | 1.066x |
| 14 | 15 | Cov + Link | 4.269959 | 0.042700 | 1.085x |
| 14 | 15 | Cov + Link + Exist, shared weights | 4.245062 | 0.042451 | 1.078x |
| 14 | 15 | Cov + Link + Exist, branch-decoupled | 4.170273 | 0.041703 | 1.059x |
| 14 | 15 | + structure-aware | 4.150552 | 0.041506 | 1.054x |
| 14 | 15 | + EMA/floor | 4.208195 | 0.042082 | 1.069x |
| 14 | 15 | + FID-FIA existence only | 10.853797 | 0.108538 | 2.757x |
| 15 | 16 | Fixed Metropolis | 4.007324 | 0.040073 | 1.000x |
| 15 | 16 | Cov only | 4.101463 | 0.041015 | 1.023x |
| 15 | 16 | Link only | 4.245238 | 0.042452 | 1.059x |
| 15 | 16 | Exist only | 4.132558 | 0.041326 | 1.031x |
| 15 | 16 | Cov + Link | 4.131899 | 0.041319 | 1.031x |
| 15 | 16 | Cov + Link + Exist, shared weights | 4.195754 | 0.041958 | 1.047x |
| 15 | 16 | Cov + Link + Exist, branch-decoupled | 4.109787 | 0.041098 | 1.026x |
| 15 | 16 | + structure-aware | 4.282559 | 0.042826 | 1.069x |
| 15 | 16 | + EMA/floor | 4.170733 | 0.041707 | 1.041x |
| 15 | 16 | + FID-FIA existence only | 10.322311 | 0.103223 | 2.576x |
| 16 | 17 | Fixed Metropolis | 4.012680 | 0.040127 | 1.000x |
| 16 | 17 | Cov only | 4.376268 | 0.043763 | 1.091x |
| 16 | 17 | Link only | 4.242112 | 0.042421 | 1.057x |
| 16 | 17 | Exist only | 4.242958 | 0.042430 | 1.057x |
| 16 | 17 | Cov + Link | 4.290749 | 0.042907 | 1.069x |
| 16 | 17 | Cov + Link + Exist, shared weights | 4.243746 | 0.042437 | 1.058x |
| 16 | 17 | Cov + Link + Exist, branch-decoupled | 4.355020 | 0.043550 | 1.085x |
| 16 | 17 | + structure-aware | 4.213240 | 0.042132 | 1.050x |
| 16 | 17 | + EMA/floor | 4.334806 | 0.043348 | 1.080x |
| 16 | 17 | + FID-FIA existence only | 10.733158 | 0.107332 | 2.675x |
| 17 | 18 | Fixed Metropolis | 3.792667 | 0.037927 | 1.000x |
| 17 | 18 | Cov only | 4.282761 | 0.042828 | 1.129x |
| 17 | 18 | Link only | 4.318966 | 0.043190 | 1.139x |
| 17 | 18 | Exist only | 4.224192 | 0.042242 | 1.114x |
| 17 | 18 | Cov + Link | 4.104129 | 0.041041 | 1.082x |
| 17 | 18 | Cov + Link + Exist, shared weights | 4.172294 | 0.041723 | 1.100x |
| 17 | 18 | Cov + Link + Exist, branch-decoupled | 4.248763 | 0.042488 | 1.120x |
| 17 | 18 | + structure-aware | 4.176248 | 0.041762 | 1.101x |
| 17 | 18 | + EMA/floor | 4.437004 | 0.044370 | 1.170x |
| 17 | 18 | + FID-FIA existence only | 10.621942 | 0.106219 | 2.801x |
| 18 | 19 | Fixed Metropolis | 3.844060 | 0.038441 | 1.000x |
| 18 | 19 | Cov only | 4.231653 | 0.042317 | 1.101x |
| 18 | 19 | Link only | 4.126883 | 0.041269 | 1.074x |
| 18 | 19 | Exist only | 4.484737 | 0.044847 | 1.167x |
| 18 | 19 | Cov + Link | 4.214435 | 0.042144 | 1.096x |
| 18 | 19 | Cov + Link + Exist, shared weights | 4.150694 | 0.041507 | 1.080x |
| 18 | 19 | Cov + Link + Exist, branch-decoupled | 4.224280 | 0.042243 | 1.099x |
| 18 | 19 | + structure-aware | 4.190414 | 0.041904 | 1.090x |
| 18 | 19 | + EMA/floor | 4.350178 | 0.043502 | 1.132x |
| 18 | 19 | + FID-FIA existence only | 11.877733 | 0.118777 | 3.090x |
| 19 | 20 | Fixed Metropolis | 3.883490 | 0.038835 | 1.000x |
| 19 | 20 | Cov only | 4.249429 | 0.042494 | 1.094x |
| 19 | 20 | Link only | 4.142767 | 0.041428 | 1.067x |
| 19 | 20 | Exist only | 5.478961 | 0.054790 | 1.411x |
| 19 | 20 | Cov + Link | 6.830866 | 0.068309 | 1.759x |
| 19 | 20 | Cov + Link + Exist, shared weights | 6.771621 | 0.067716 | 1.744x |
| 19 | 20 | Cov + Link + Exist, branch-decoupled | 4.203565 | 0.042036 | 1.082x |
| 19 | 20 | + structure-aware | 4.284368 | 0.042844 | 1.103x |
| 19 | 20 | + EMA/floor | 4.203437 | 0.042034 | 1.082x |
| 19 | 20 | + FID-FIA existence only | 10.698230 | 0.106982 | 2.755x |
| 20 | 21 | Fixed Metropolis | 3.925101 | 0.039251 | 1.000x |
| 20 | 21 | Cov only | 4.304019 | 0.043040 | 1.097x |
| 20 | 21 | Link only | 4.134125 | 0.041341 | 1.053x |
| 20 | 21 | Exist only | 4.131534 | 0.041315 | 1.053x |
| 20 | 21 | Cov + Link | 4.175172 | 0.041752 | 1.064x |
| 20 | 21 | Cov + Link + Exist, shared weights | 4.190414 | 0.041904 | 1.068x |
| 20 | 21 | Cov + Link + Exist, branch-decoupled | 4.171373 | 0.041714 | 1.063x |
| 20 | 21 | + structure-aware | 4.368702 | 0.043687 | 1.113x |
| 20 | 21 | + EMA/floor | 4.110460 | 0.041105 | 1.047x |
| 20 | 21 | + FID-FIA existence only | 10.761870 | 0.107619 | 2.742x |
| 21 | 22 | Fixed Metropolis | 4.075613 | 0.040756 | 1.000x |
| 21 | 22 | Cov only | 4.365032 | 0.043650 | 1.071x |
| 21 | 22 | Link only | 4.094270 | 0.040943 | 1.005x |
| 21 | 22 | Exist only | 4.423842 | 0.044238 | 1.085x |
| 21 | 22 | Cov + Link | 4.164332 | 0.041643 | 1.022x |
| 21 | 22 | Cov + Link + Exist, shared weights | 4.302902 | 0.043029 | 1.056x |
| 21 | 22 | Cov + Link + Exist, branch-decoupled | 4.231569 | 0.042316 | 1.038x |
| 21 | 22 | + structure-aware | 4.362796 | 0.043628 | 1.070x |
| 21 | 22 | + EMA/floor | 4.217286 | 0.042173 | 1.035x |
| 21 | 22 | + FID-FIA existence only | 11.132329 | 0.111323 | 2.731x |
| 22 | 23 | Fixed Metropolis | 3.850458 | 0.038505 | 1.000x |
| 22 | 23 | Cov only | 4.071909 | 0.040719 | 1.058x |
| 22 | 23 | Link only | 4.109877 | 0.041099 | 1.067x |
| 22 | 23 | Exist only | 4.133475 | 0.041335 | 1.074x |
| 22 | 23 | Cov + Link | 4.070110 | 0.040701 | 1.057x |
| 22 | 23 | Cov + Link + Exist, shared weights | 4.348279 | 0.043483 | 1.129x |
| 22 | 23 | Cov + Link + Exist, branch-decoupled | 4.240269 | 0.042403 | 1.101x |
| 22 | 23 | + structure-aware | 4.178988 | 0.041790 | 1.085x |
| 22 | 23 | + EMA/floor | 4.159666 | 0.041597 | 1.080x |
| 22 | 23 | + FID-FIA existence only | 10.365379 | 0.103654 | 2.692x |
| 23 | 24 | Fixed Metropolis | 3.970009 | 0.039700 | 1.000x |
| 23 | 24 | Cov only | 4.230765 | 0.042308 | 1.066x |
| 23 | 24 | Link only | 4.124616 | 0.041246 | 1.039x |
| 23 | 24 | Exist only | 4.320367 | 0.043204 | 1.088x |
| 23 | 24 | Cov + Link | 4.328168 | 0.043282 | 1.090x |
| 23 | 24 | Cov + Link + Exist, shared weights | 4.213641 | 0.042136 | 1.061x |
| 23 | 24 | Cov + Link + Exist, branch-decoupled | 4.187985 | 0.041880 | 1.055x |
| 23 | 24 | + structure-aware | 4.159920 | 0.041599 | 1.048x |
| 23 | 24 | + EMA/floor | 4.133729 | 0.041337 | 1.041x |
| 23 | 24 | + FID-FIA existence only | 10.294404 | 0.102944 | 2.593x |
| 24 | 25 | Fixed Metropolis | 3.780932 | 0.037809 | 1.000x |
| 24 | 25 | Cov only | 4.079792 | 0.040798 | 1.079x |
| 24 | 25 | Link only | 4.049411 | 0.040494 | 1.071x |
| 24 | 25 | Exist only | 4.020603 | 0.040206 | 1.063x |
| 24 | 25 | Cov + Link | 4.052579 | 0.040526 | 1.072x |
| 24 | 25 | Cov + Link + Exist, shared weights | 4.233698 | 0.042337 | 1.120x |
| 24 | 25 | Cov + Link + Exist, branch-decoupled | 4.121897 | 0.041219 | 1.090x |
| 24 | 25 | + structure-aware | 4.083328 | 0.040833 | 1.080x |
| 24 | 25 | + EMA/floor | 4.045899 | 0.040459 | 1.070x |
| 24 | 25 | + FID-FIA existence only | 10.690627 | 0.106906 | 2.828x |
| 25 | 26 | Fixed Metropolis | 3.782673 | 0.037827 | 1.000x |
| 25 | 26 | Cov only | 4.074009 | 0.040740 | 1.077x |
| 25 | 26 | Link only | 4.023989 | 0.040240 | 1.064x |
| 25 | 26 | Exist only | 4.089016 | 0.040890 | 1.081x |
| 25 | 26 | Cov + Link | 4.081399 | 0.040814 | 1.079x |
| 25 | 26 | Cov + Link + Exist, shared weights | 4.064920 | 0.040649 | 1.075x |
| 25 | 26 | Cov + Link + Exist, branch-decoupled | 4.048188 | 0.040482 | 1.070x |
| 25 | 26 | + structure-aware | 4.048470 | 0.040485 | 1.070x |
| 25 | 26 | + EMA/floor | 4.023347 | 0.040233 | 1.064x |
| 25 | 26 | + FID-FIA existence only | 10.617464 | 0.106175 | 2.807x |
| 26 | 27 | Fixed Metropolis | 3.684651 | 0.036847 | 1.000x |
| 26 | 27 | Cov only | 3.995908 | 0.039959 | 1.084x |
| 26 | 27 | Link only | 3.928804 | 0.039288 | 1.066x |
| 26 | 27 | Exist only | 4.037876 | 0.040379 | 1.096x |
| 26 | 27 | Cov + Link | 6.509956 | 0.065100 | 1.767x |
| 26 | 27 | Cov + Link + Exist, shared weights | 7.332891 | 0.073329 | 1.990x |
| 26 | 27 | Cov + Link + Exist, branch-decoupled | 6.576207 | 0.065762 | 1.785x |
| 26 | 27 | + structure-aware | 6.520143 | 0.065201 | 1.770x |
| 26 | 27 | + EMA/floor | 6.324536 | 0.063245 | 1.716x |
| 26 | 27 | + FID-FIA existence only | 15.945637 | 0.159456 | 4.328x |
| 27 | 28 | Fixed Metropolis | 5.925783 | 0.059258 | 1.000x |
| 27 | 28 | Cov only | 6.396737 | 0.063967 | 1.079x |
| 27 | 28 | Link only | 6.318999 | 0.063190 | 1.066x |
| 27 | 28 | Exist only | 6.477806 | 0.064778 | 1.093x |
| 27 | 28 | Cov + Link | 6.274983 | 0.062750 | 1.059x |
| 27 | 28 | Cov + Link + Exist, shared weights | 7.032945 | 0.070329 | 1.187x |
| 27 | 28 | Cov + Link + Exist, branch-decoupled | 6.995912 | 0.069959 | 1.181x |
| 27 | 28 | + structure-aware | 6.916000 | 0.069160 | 1.167x |
| 27 | 28 | + EMA/floor | 7.287375 | 0.072874 | 1.230x |
| 27 | 28 | + FID-FIA existence only | 18.323717 | 0.183237 | 3.092x |
| 28 | 29 | Fixed Metropolis | 7.116349 | 0.071163 | 1.000x |
| 28 | 29 | Cov only | 7.716973 | 0.077170 | 1.084x |
| 28 | 29 | Link only | 7.589092 | 0.075891 | 1.066x |
| 28 | 29 | Exist only | 7.715518 | 0.077155 | 1.084x |
| 28 | 29 | Cov + Link | 7.603131 | 0.076031 | 1.068x |
| 28 | 29 | Cov + Link + Exist, shared weights | 7.672219 | 0.076722 | 1.078x |
| 28 | 29 | Cov + Link + Exist, branch-decoupled | 7.672015 | 0.076720 | 1.078x |
| 28 | 29 | + structure-aware | 7.724287 | 0.077243 | 1.085x |
| 28 | 29 | + EMA/floor | 7.707790 | 0.077078 | 1.083x |
| 28 | 29 | + FID-FIA existence only | 19.224290 | 0.192243 | 2.701x |
| 29 | 30 | Fixed Metropolis | 6.936247 | 0.069362 | 1.000x |
| 29 | 30 | Cov only | 7.661523 | 0.076615 | 1.105x |
| 29 | 30 | Link only | 7.259037 | 0.072590 | 1.047x |
| 29 | 30 | Exist only | 7.611248 | 0.076112 | 1.097x |
| 29 | 30 | Cov + Link | 7.201912 | 0.072019 | 1.038x |
| 29 | 30 | Cov + Link + Exist, shared weights | 7.225932 | 0.072259 | 1.042x |
| 29 | 30 | Cov + Link + Exist, branch-decoupled | 7.275358 | 0.072754 | 1.049x |
| 29 | 30 | + structure-aware | 7.255292 | 0.072553 | 1.046x |
| 29 | 30 | + EMA/floor | 7.340765 | 0.073408 | 1.058x |
| 29 | 30 | + FID-FIA existence only | 18.702435 | 0.187024 | 2.696x |
| 30 | 31 | Fixed Metropolis | 7.011329 | 0.070113 | 1.000x |
| 30 | 31 | Cov only | 7.494967 | 0.074950 | 1.069x |
| 30 | 31 | Link only | 7.307436 | 0.073074 | 1.042x |
| 30 | 31 | Exist only | 7.490416 | 0.074904 | 1.068x |
| 30 | 31 | Cov + Link | 7.338562 | 0.073386 | 1.047x |
| 30 | 31 | Cov + Link + Exist, shared weights | 7.423884 | 0.074239 | 1.059x |
| 30 | 31 | Cov + Link + Exist, branch-decoupled | 7.378700 | 0.073787 | 1.052x |
| 30 | 31 | + structure-aware | 7.375004 | 0.073750 | 1.052x |
| 30 | 31 | + EMA/floor | 7.364371 | 0.073644 | 1.050x |
| 30 | 31 | + FID-FIA existence only | 18.415091 | 0.184151 | 2.626x |
| 31 | 32 | Fixed Metropolis | 6.645671 | 0.066457 | 1.000x |
| 31 | 32 | Cov only | 7.321891 | 0.073219 | 1.102x |
| 31 | 32 | Link only | 7.352219 | 0.073522 | 1.106x |
| 31 | 32 | Exist only | 7.301876 | 0.073019 | 1.099x |
| 31 | 32 | Cov + Link | 7.323696 | 0.073237 | 1.102x |
| 31 | 32 | Cov + Link + Exist, shared weights | 7.367878 | 0.073679 | 1.109x |
| 31 | 32 | Cov + Link + Exist, branch-decoupled | 7.444159 | 0.074442 | 1.120x |
| 31 | 32 | + structure-aware | 7.451608 | 0.074516 | 1.121x |
| 31 | 32 | + EMA/floor | 7.402763 | 0.074028 | 1.114x |
| 31 | 32 | + FID-FIA existence only | 19.376363 | 0.193764 | 2.916x |
| 32 | 33 | Fixed Metropolis | 6.897856 | 0.068979 | 1.000x |
| 32 | 33 | Cov only | 7.502077 | 0.075021 | 1.088x |
| 32 | 33 | Link only | 7.488008 | 0.074880 | 1.086x |
| 32 | 33 | Exist only | 7.533865 | 0.075339 | 1.092x |
| 32 | 33 | Cov + Link | 7.524425 | 0.075244 | 1.091x |
| 32 | 33 | Cov + Link + Exist, shared weights | 7.456658 | 0.074567 | 1.081x |
| 32 | 33 | Cov + Link + Exist, branch-decoupled | 7.472606 | 0.074726 | 1.083x |
| 32 | 33 | + structure-aware | 7.517856 | 0.075179 | 1.090x |
| 32 | 33 | + EMA/floor | 7.557153 | 0.075572 | 1.096x |
| 32 | 33 | + FID-FIA existence only | 14.679989 | 0.146800 | 2.128x |
| 33 | 34 | Fixed Metropolis | 4.238303 | 0.042383 | 1.000x |
| 33 | 34 | Cov only | 4.477177 | 0.044772 | 1.056x |
| 33 | 34 | Link only | 4.341345 | 0.043413 | 1.024x |
| 33 | 34 | Exist only | 4.478101 | 0.044781 | 1.057x |
| 33 | 34 | Cov + Link | 4.345302 | 0.043453 | 1.025x |
| 33 | 34 | Cov + Link + Exist, shared weights | 4.290211 | 0.042902 | 1.012x |
| 33 | 34 | Cov + Link + Exist, branch-decoupled | 4.300483 | 0.043005 | 1.015x |
| 33 | 34 | + structure-aware | 4.287520 | 0.042875 | 1.012x |
| 33 | 34 | + EMA/floor | 4.607191 | 0.046072 | 1.087x |
| 33 | 34 | + FID-FIA existence only | 11.286251 | 0.112863 | 2.663x |
| 34 | 35 | Fixed Metropolis | 4.018778 | 0.040188 | 1.000x |
| 34 | 35 | Cov only | 4.365536 | 0.043655 | 1.086x |
| 34 | 35 | Link only | 4.350427 | 0.043504 | 1.083x |
| 34 | 35 | Exist only | 4.446426 | 0.044464 | 1.106x |
| 34 | 35 | Cov + Link | 4.251998 | 0.042520 | 1.058x |
| 34 | 35 | Cov + Link + Exist, shared weights | 4.378946 | 0.043789 | 1.090x |
| 34 | 35 | Cov + Link + Exist, branch-decoupled | 4.394339 | 0.043943 | 1.093x |
| 34 | 35 | + structure-aware | 4.467914 | 0.044679 | 1.112x |
| 34 | 35 | + EMA/floor | 4.470550 | 0.044705 | 1.112x |
| 34 | 35 | + FID-FIA existence only | 11.441069 | 0.114411 | 2.847x |
| 35 | 36 | Fixed Metropolis | 3.964443 | 0.039644 | 1.000x |
| 35 | 36 | Cov only | 4.273028 | 0.042730 | 1.078x |
| 35 | 36 | Link only | 4.242573 | 0.042426 | 1.070x |
| 35 | 36 | Exist only | 4.355208 | 0.043552 | 1.099x |
| 35 | 36 | Cov + Link | 4.187365 | 0.041874 | 1.056x |
| 35 | 36 | Cov + Link + Exist, shared weights | 4.229847 | 0.042298 | 1.067x |
| 35 | 36 | Cov + Link + Exist, branch-decoupled | 4.177190 | 0.041772 | 1.054x |
| 35 | 36 | + structure-aware | 4.226165 | 0.042262 | 1.066x |
| 35 | 36 | + EMA/floor | 4.179718 | 0.041797 | 1.054x |
| 35 | 36 | + FID-FIA existence only | 10.896229 | 0.108962 | 2.748x |
| 36 | 37 | Fixed Metropolis | 4.168562 | 0.041686 | 1.000x |
| 36 | 37 | Cov only | 4.391820 | 0.043918 | 1.054x |
| 36 | 37 | Link only | 4.475630 | 0.044756 | 1.074x |
| 36 | 37 | Exist only | 4.456845 | 0.044568 | 1.069x |
| 36 | 37 | Cov + Link | 4.546937 | 0.045469 | 1.091x |
| 36 | 37 | Cov + Link + Exist, shared weights | 4.623908 | 0.046239 | 1.109x |
| 36 | 37 | Cov + Link + Exist, branch-decoupled | 4.526475 | 0.045265 | 1.086x |
| 36 | 37 | + structure-aware | 4.495433 | 0.044954 | 1.078x |
| 36 | 37 | + EMA/floor | 4.460238 | 0.044602 | 1.070x |
| 36 | 37 | + FID-FIA existence only | 11.027718 | 0.110277 | 2.645x |
| 37 | 38 | Fixed Metropolis | 4.043905 | 0.040439 | 1.000x |
| 37 | 38 | Cov only | 4.567106 | 0.045671 | 1.129x |
| 37 | 38 | Link only | 4.252923 | 0.042529 | 1.052x |
| 37 | 38 | Exist only | 4.417756 | 0.044178 | 1.092x |
| 37 | 38 | Cov + Link | 4.345638 | 0.043456 | 1.075x |
| 37 | 38 | Cov + Link + Exist, shared weights | 4.403358 | 0.044034 | 1.089x |
| 37 | 38 | Cov + Link + Exist, branch-decoupled | 4.380257 | 0.043803 | 1.083x |
| 37 | 38 | + structure-aware | 4.350353 | 0.043504 | 1.076x |
| 37 | 38 | + EMA/floor | 4.497629 | 0.044976 | 1.112x |
| 37 | 38 | + FID-FIA existence only | 11.021674 | 0.110217 | 2.726x |
| 38 | 39 | Fixed Metropolis | 4.073071 | 0.040731 | 1.000x |
| 38 | 39 | Cov only | 4.442459 | 0.044425 | 1.091x |
| 38 | 39 | Link only | 4.395187 | 0.043952 | 1.079x |
| 38 | 39 | Exist only | 4.462706 | 0.044627 | 1.096x |
| 38 | 39 | Cov + Link | 4.550145 | 0.045501 | 1.117x |
| 38 | 39 | Cov + Link + Exist, shared weights | 4.377137 | 0.043771 | 1.075x |
| 38 | 39 | Cov + Link + Exist, branch-decoupled | 4.676468 | 0.046765 | 1.148x |
| 38 | 39 | + structure-aware | 4.489597 | 0.044896 | 1.102x |
| 38 | 39 | + EMA/floor | 4.542073 | 0.045421 | 1.115x |
| 38 | 39 | + FID-FIA existence only | 11.298433 | 0.112984 | 2.774x |
| 39 | 40 | Fixed Metropolis | 4.080782 | 0.040808 | 1.000x |
| 39 | 40 | Cov only | 4.459360 | 0.044594 | 1.093x |
| 39 | 40 | Link only | 4.497879 | 0.044979 | 1.102x |
| 39 | 40 | Exist only | 4.526088 | 0.045261 | 1.109x |
| 39 | 40 | Cov + Link | 4.327667 | 0.043277 | 1.060x |
| 39 | 40 | Cov + Link + Exist, shared weights | 4.475969 | 0.044760 | 1.097x |
| 39 | 40 | Cov + Link + Exist, branch-decoupled | 4.276147 | 0.042761 | 1.048x |
| 39 | 40 | + structure-aware | 4.467017 | 0.044670 | 1.095x |
| 39 | 40 | + EMA/floor | 4.513541 | 0.045135 | 1.106x |
| 39 | 40 | + FID-FIA existence only | 11.344487 | 0.113445 | 2.780x |
| 40 | 41 | Fixed Metropolis | 4.107589 | 0.041076 | 1.000x |
| 40 | 41 | Cov only | 4.478278 | 0.044783 | 1.090x |
| 40 | 41 | Link only | 4.342703 | 0.043427 | 1.057x |
| 40 | 41 | Exist only | 4.376690 | 0.043767 | 1.066x |
| 40 | 41 | Cov + Link | 4.269078 | 0.042691 | 1.039x |
| 40 | 41 | Cov + Link + Exist, shared weights | 4.401764 | 0.044018 | 1.072x |
| 40 | 41 | Cov + Link + Exist, branch-decoupled | 4.398815 | 0.043988 | 1.071x |
| 40 | 41 | + structure-aware | 4.499024 | 0.044990 | 1.095x |
| 40 | 41 | + EMA/floor | 4.523014 | 0.045230 | 1.101x |
| 40 | 41 | + FID-FIA existence only | 11.051103 | 0.110511 | 2.690x |
| 41 | 42 | Fixed Metropolis | 4.046480 | 0.040465 | 1.000x |
| 41 | 42 | Cov only | 4.459008 | 0.044590 | 1.102x |
| 41 | 42 | Link only | 4.490276 | 0.044903 | 1.110x |
| 41 | 42 | Exist only | 4.535190 | 0.045352 | 1.121x |
| 41 | 42 | Cov + Link | 4.398887 | 0.043989 | 1.087x |
| 41 | 42 | Cov + Link + Exist, shared weights | 4.331865 | 0.043319 | 1.071x |
| 41 | 42 | Cov + Link + Exist, branch-decoupled | 4.586810 | 0.045868 | 1.134x |
| 41 | 42 | + structure-aware | 4.399501 | 0.043995 | 1.087x |
| 41 | 42 | + EMA/floor | 4.561972 | 0.045620 | 1.127x |
| 41 | 42 | + FID-FIA existence only | 11.327679 | 0.113277 | 2.799x |
| 42 | 43 | Fixed Metropolis | 4.038805 | 0.040388 | 1.000x |
| 42 | 43 | Cov only | 4.493106 | 0.044931 | 1.112x |
| 42 | 43 | Link only | 4.301198 | 0.043012 | 1.065x |
| 42 | 43 | Exist only | 4.618370 | 0.046184 | 1.143x |
| 42 | 43 | Cov + Link | 4.320467 | 0.043205 | 1.070x |
| 42 | 43 | Cov + Link + Exist, shared weights | 4.276244 | 0.042762 | 1.059x |
| 42 | 43 | Cov + Link + Exist, branch-decoupled | 4.245732 | 0.042457 | 1.051x |
| 42 | 43 | + structure-aware | 4.297361 | 0.042974 | 1.064x |
| 42 | 43 | + EMA/floor | 4.403719 | 0.044037 | 1.090x |
| 42 | 43 | + FID-FIA existence only | 11.181433 | 0.111814 | 2.769x |
| 43 | 44 | Fixed Metropolis | 4.103774 | 0.041038 | 1.000x |
| 43 | 44 | Cov only | 4.368112 | 0.043681 | 1.064x |
| 43 | 44 | Link only | 4.214103 | 0.042141 | 1.027x |
| 43 | 44 | Exist only | 4.414541 | 0.044145 | 1.076x |
| 43 | 44 | Cov + Link | 4.345311 | 0.043453 | 1.059x |
| 43 | 44 | Cov + Link + Exist, shared weights | 4.274276 | 0.042743 | 1.042x |
| 43 | 44 | Cov + Link + Exist, branch-decoupled | 4.203677 | 0.042037 | 1.024x |
| 43 | 44 | + structure-aware | 4.326895 | 0.043269 | 1.054x |
| 43 | 44 | + EMA/floor | 4.328852 | 0.043289 | 1.055x |
| 43 | 44 | + FID-FIA existence only | 11.453406 | 0.114534 | 2.791x |
| 44 | 45 | Fixed Metropolis | 3.869213 | 0.038692 | 1.000x |
| 44 | 45 | Cov only | 4.301152 | 0.043012 | 1.112x |
| 44 | 45 | Link only | 4.487051 | 0.044871 | 1.160x |
| 44 | 45 | Exist only | 4.370099 | 0.043701 | 1.129x |
| 44 | 45 | Cov + Link | 4.300507 | 0.043005 | 1.111x |
| 44 | 45 | Cov + Link + Exist, shared weights | 4.277590 | 0.042776 | 1.106x |
| 44 | 45 | Cov + Link + Exist, branch-decoupled | 4.397397 | 0.043974 | 1.137x |
| 44 | 45 | + structure-aware | 4.387060 | 0.043871 | 1.134x |
| 44 | 45 | + EMA/floor | 4.282829 | 0.042828 | 1.107x |
| 44 | 45 | + FID-FIA existence only | 11.065759 | 0.110658 | 2.860x |
| 45 | 46 | Fixed Metropolis | 4.272124 | 0.042721 | 1.000x |
| 45 | 46 | Cov only | 4.491461 | 0.044915 | 1.051x |
| 45 | 46 | Link only | 4.433132 | 0.044331 | 1.038x |
| 45 | 46 | Exist only | 4.545138 | 0.045451 | 1.064x |
| 45 | 46 | Cov + Link | 4.405570 | 0.044056 | 1.031x |
| 45 | 46 | Cov + Link + Exist, shared weights | 4.545950 | 0.045460 | 1.064x |
| 45 | 46 | Cov + Link + Exist, branch-decoupled | 4.487246 | 0.044872 | 1.050x |
| 45 | 46 | + structure-aware | 4.470614 | 0.044706 | 1.046x |
| 45 | 46 | + EMA/floor | 4.595131 | 0.045951 | 1.076x |
| 45 | 46 | + FID-FIA existence only | 13.085127 | 0.130851 | 3.063x |
| 46 | 47 | Fixed Metropolis | 6.872929 | 0.068729 | 1.000x |
| 46 | 47 | Cov only | 7.103216 | 0.071032 | 1.034x |
| 46 | 47 | Link only | 6.972685 | 0.069727 | 1.015x |
| 46 | 47 | Exist only | 7.497915 | 0.074979 | 1.091x |
| 46 | 47 | Cov + Link | 4.221299 | 0.042213 | 0.614x |
| 46 | 47 | Cov + Link + Exist, shared weights | 5.743611 | 0.057436 | 0.836x |
| 46 | 47 | Cov + Link + Exist, branch-decoupled | 7.360044 | 0.073600 | 1.071x |
| 46 | 47 | + structure-aware | 7.156469 | 0.071565 | 1.041x |
| 46 | 47 | + EMA/floor | 7.105535 | 0.071055 | 1.034x |
| 46 | 47 | + FID-FIA existence only | 11.349808 | 0.113498 | 1.651x |
| 47 | 48 | Fixed Metropolis | 3.774303 | 0.037743 | 1.000x |
| 47 | 48 | Cov only | 4.171357 | 0.041714 | 1.105x |
| 47 | 48 | Link only | 5.569116 | 0.055691 | 1.476x |
| 47 | 48 | Exist only | 6.887068 | 0.068871 | 1.825x |
| 47 | 48 | Cov + Link | 6.747500 | 0.067475 | 1.788x |
| 47 | 48 | Cov + Link + Exist, shared weights | 5.269816 | 0.052698 | 1.396x |
| 47 | 48 | Cov + Link + Exist, branch-decoupled | 4.069879 | 0.040699 | 1.078x |
| 47 | 48 | + structure-aware | 4.082857 | 0.040829 | 1.082x |
| 47 | 48 | + EMA/floor | 4.060643 | 0.040606 | 1.076x |
| 47 | 48 | + FID-FIA existence only | 12.351613 | 0.123516 | 3.273x |
| 48 | 49 | Fixed Metropolis | 3.699659 | 0.036997 | 1.000x |
| 48 | 49 | Cov only | 4.086800 | 0.040868 | 1.105x |
| 48 | 49 | Link only | 4.012277 | 0.040123 | 1.084x |
| 48 | 49 | Exist only | 4.101300 | 0.041013 | 1.109x |
| 48 | 49 | Cov + Link | 4.036718 | 0.040367 | 1.091x |
| 48 | 49 | Cov + Link + Exist, shared weights | 3.995389 | 0.039954 | 1.080x |
| 48 | 49 | Cov + Link + Exist, branch-decoupled | 4.044337 | 0.040443 | 1.093x |
| 48 | 49 | + structure-aware | 3.977707 | 0.039777 | 1.075x |
| 48 | 49 | + EMA/floor | 4.004412 | 0.040044 | 1.082x |
| 48 | 49 | + FID-FIA existence only | 10.327919 | 0.103279 | 2.792x |
| 49 | 50 | Fixed Metropolis | 3.691429 | 0.036914 | 1.000x |
| 49 | 50 | Cov only | 4.184992 | 0.041850 | 1.134x |
| 49 | 50 | Link only | 4.023318 | 0.040233 | 1.090x |
| 49 | 50 | Exist only | 4.024565 | 0.040246 | 1.090x |
| 49 | 50 | Cov + Link | 3.999834 | 0.039998 | 1.084x |
| 49 | 50 | Cov + Link + Exist, shared weights | 4.008012 | 0.040080 | 1.086x |
| 49 | 50 | Cov + Link + Exist, branch-decoupled | 4.027304 | 0.040273 | 1.091x |
| 49 | 50 | + structure-aware | 4.049462 | 0.040495 | 1.097x |
| 49 | 50 | + EMA/floor | 4.008851 | 0.040089 | 1.086x |
| 49 | 50 | + FID-FIA existence only | 10.247512 | 0.102475 | 2.776x |
| 50 | 51 | Fixed Metropolis | 3.682236 | 0.036822 | 1.000x |
| 50 | 51 | Cov only | 4.025844 | 0.040258 | 1.093x |
| 50 | 51 | Link only | 3.904421 | 0.039044 | 1.060x |
| 50 | 51 | Exist only | 4.022226 | 0.040222 | 1.092x |
| 50 | 51 | Cov + Link | 3.918375 | 0.039184 | 1.064x |
| 50 | 51 | Cov + Link + Exist, shared weights | 3.929622 | 0.039296 | 1.067x |
| 50 | 51 | Cov + Link + Exist, branch-decoupled | 4.018337 | 0.040183 | 1.091x |
| 50 | 51 | + structure-aware | 3.947231 | 0.039472 | 1.072x |
| 50 | 51 | + EMA/floor | 4.001186 | 0.040012 | 1.087x |
| 50 | 51 | + FID-FIA existence only | 10.796004 | 0.107960 | 2.932x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Fixed Metropolis | 2.781144 | 1.630027 | 1.364300 |
| Cov only | 2.556472 | 1.567091 | 0.981375 |
| Link only | 2.073760 | 1.625951 | 0.300100 |
| Exist only | 2.640868 | 1.565575 | 1.109800 |
| Cov + Link | 2.063938 | 1.632838 | 0.279875 |
| Cov + Link + Exist, shared weights | 2.067611 | 1.633944 | 0.281275 |
| Cov + Link + Exist, branch-decoupled | 2.068470 | 1.633364 | 0.283100 |
| + structure-aware | 2.071760 | 1.636237 | 0.282850 |
| + EMA/floor | 2.292981 | 1.601289 | 0.550925 |
| + FID-FIA existence only | 2.244638 | 1.604295 | 0.483475 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Fixed Metropolis | E-OSPA | 2.781144 +/- 0.128910 | [2.745412, 2.816876] | 50 |
| Cov only | E-OSPA | 2.556472 +/- 0.106473 | [2.526960, 2.585985] | 50 |
| Link only | E-OSPA | 2.073760 +/- 0.052950 | [2.059083, 2.088437] | 50 |
| Exist only | E-OSPA | 2.640868 +/- 0.120922 | [2.607350, 2.674386] | 50 |
| Cov + Link | E-OSPA | 2.063938 +/- 0.050941 | [2.049818, 2.078058] | 50 |
| Cov + Link + Exist, shared weights | E-OSPA | 2.067611 +/- 0.051163 | [2.053429, 2.081793] | 50 |
| Cov + Link + Exist, branch-decoupled | E-OSPA | 2.068470 +/- 0.051614 | [2.054163, 2.082777] | 50 |
| + structure-aware | E-OSPA | 2.071760 +/- 0.051491 | [2.057488, 2.086033] | 50 |
| + EMA/floor | E-OSPA | 2.292981 +/- 0.081787 | [2.270311, 2.315652] | 50 |
| + FID-FIA existence only | E-OSPA | 2.244638 +/- 0.081065 | [2.222168, 2.267108] | 50 |
| Fixed Metropolis | RMSE | 1.630027 +/- 0.049051 | [1.616431, 1.643623] | 50 |
| Cov only | RMSE | 1.567091 +/- 0.048648 | [1.553607, 1.580576] | 50 |
| Link only | RMSE | 1.625951 +/- 0.044979 | [1.613483, 1.638419] | 50 |
| Exist only | RMSE | 1.565575 +/- 0.052360 | [1.551062, 1.580089] | 50 |
| Cov + Link | RMSE | 1.632838 +/- 0.044151 | [1.620600, 1.645076] | 50 |
| Cov + Link + Exist, shared weights | RMSE | 1.633944 +/- 0.043354 | [1.621927, 1.645961] | 50 |
| Cov + Link + Exist, branch-decoupled | RMSE | 1.633364 +/- 0.043551 | [1.621292, 1.645435] | 50 |
| + structure-aware | RMSE | 1.636237 +/- 0.045427 | [1.623646, 1.648829] | 50 |
| + EMA/floor | RMSE | 1.601289 +/- 0.048619 | [1.587812, 1.614765] | 50 |
| + FID-FIA existence only | RMSE | 1.604295 +/- 0.045892 | [1.591574, 1.617015] | 50 |
| Fixed Metropolis | CardErr | 1.364300 +/- 0.260181 | [1.292181, 1.436419] | 50 |
| Cov only | CardErr | 0.981375 +/- 0.154912 | [0.938435, 1.024315] | 50 |
| Link only | CardErr | 0.300100 +/- 0.047958 | [0.286807, 0.313393] | 50 |
| Exist only | CardErr | 1.109800 +/- 0.200757 | [1.054153, 1.165447] | 50 |
| Cov + Link | CardErr | 0.279875 +/- 0.044848 | [0.267444, 0.292306] | 50 |
| Cov + Link + Exist, shared weights | CardErr | 0.281275 +/- 0.045764 | [0.268590, 0.293960] | 50 |
| Cov + Link + Exist, branch-decoupled | CardErr | 0.283100 +/- 0.045803 | [0.270404, 0.295796] | 50 |
| + structure-aware | CardErr | 0.282850 +/- 0.045723 | [0.270176, 0.295524] | 50 |
| + EMA/floor | CardErr | 0.550925 +/- 0.084961 | [0.527375, 0.574475] | 50 |
| + FID-FIA existence only | CardErr | 0.483475 +/- 0.078067 | [0.461836, 0.505114] | 50 |

## Paired Local-Metric Improvements Relative to Fixed Metropolis
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Cov only | E-OSPA | 0.224672 +/- 0.039393 | [0.213753, 0.235591] | 8.08% | 50/50 | 1.776e-15 |
| Link only | E-OSPA | 0.707384 +/- 0.101632 | [0.679213, 0.735555] | 25.44% | 50/50 | 1.776e-15 |
| Exist only | E-OSPA | 0.140276 +/- 0.031045 | [0.131671, 0.148881] | 5.04% | 50/50 | 1.776e-15 |
| Cov + Link | E-OSPA | 0.717206 +/- 0.102343 | [0.688838, 0.745574] | 25.79% | 50/50 | 1.776e-15 |
| Cov + Link + Exist, shared weights | E-OSPA | 0.713533 +/- 0.102589 | [0.685097, 0.741970] | 25.66% | 50/50 | 1.776e-15 |
| Cov + Link + Exist, branch-decoupled | E-OSPA | 0.712674 +/- 0.102164 | [0.684356, 0.740993] | 25.63% | 50/50 | 1.776e-15 |
| + structure-aware | E-OSPA | 0.709384 +/- 0.102041 | [0.681100, 0.737668] | 25.51% | 50/50 | 1.776e-15 |
| + EMA/floor | E-OSPA | 0.488163 +/- 0.077415 | [0.466704, 0.509621] | 17.55% | 50/50 | 1.776e-15 |
| + FID-FIA existence only | E-OSPA | 0.536506 +/- 0.088281 | [0.512036, 0.560976] | 19.29% | 50/50 | 1.776e-15 |
| Cov only | RMSE | 0.062936 +/- 0.031373 | [0.054240, 0.071632] | 3.86% | 50/50 | 1.776e-15 |
| Link only | RMSE | 0.004076 +/- 0.035490 | [-0.005761, 0.013913] | 0.25% | 26/50 | 0.8877 |
| Exist only | RMSE | 0.064452 +/- 0.032555 | [0.055428, 0.073475] | 3.95% | 48/50 | 2.267e-12 |
| Cov + Link | RMSE | -0.002811 +/- 0.035936 | [-0.012772, 0.007150] | -0.17% | 20/50 | 0.2026 |
| Cov + Link + Exist, shared weights | RMSE | -0.003917 +/- 0.035733 | [-0.013821, 0.005988] | -0.24% | 19/50 | 0.1189 |
| Cov + Link + Exist, branch-decoupled | RMSE | -0.003336 +/- 0.035878 | [-0.013281, 0.006608] | -0.20% | 19/50 | 0.1189 |
| + structure-aware | RMSE | -0.006210 +/- 0.035433 | [-0.016032, 0.003611] | -0.38% | 20/50 | 0.2026 |
| + EMA/floor | RMSE | 0.028738 +/- 0.032973 | [0.019599, 0.037878] | 1.76% | 42/50 | 1.164e-06 |
| + FID-FIA existence only | RMSE | 0.025733 +/- 0.033997 | [0.016309, 0.035156] | 1.58% | 40/50 | 2.386e-05 |
| Cov only | CardErr | 0.382925 +/- 0.127313 | [0.347636, 0.418214] | 28.07% | 50/50 | 1.776e-15 |
| Link only | CardErr | 1.064200 +/- 0.248119 | [0.995425, 1.132975] | 78.00% | 50/50 | 1.776e-15 |
| Exist only | CardErr | 0.254500 +/- 0.094599 | [0.228279, 0.280721] | 18.65% | 50/50 | 1.776e-15 |
| Cov + Link | CardErr | 1.084425 +/- 0.248027 | [1.015675, 1.153175] | 79.49% | 50/50 | 1.776e-15 |
| Cov + Link + Exist, shared weights | CardErr | 1.083025 +/- 0.248663 | [1.014099, 1.151951] | 79.38% | 50/50 | 1.776e-15 |
| Cov + Link + Exist, branch-decoupled | CardErr | 1.081200 +/- 0.248922 | [1.012202, 1.150198] | 79.25% | 50/50 | 1.776e-15 |
| + structure-aware | CardErr | 1.081450 +/- 0.248141 | [1.012669, 1.150231] | 79.27% | 50/50 | 1.776e-15 |
| + EMA/floor | CardErr | 0.813375 +/- 0.207891 | [0.755751, 0.870999] | 59.62% | 50/50 | 1.776e-15 |
| + FID-FIA existence only | CardErr | 0.880825 +/- 0.217736 | [0.820472, 0.941178] | 64.56% | 50/50 | 1.776e-15 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | Fixed Metropolis | 2.644619 | 1.602249 | 1.017000 |
| 1 | Cov only | 2.429722 | 1.520354 | 0.746200 |
| 1 | Link only | 2.028961 | 1.589296 | 0.291600 |
| 1 | Exist only | 2.481606 | 1.513905 | 0.799800 |
| 1 | Cov + Link | 2.017294 | 1.595492 | 0.268200 |
| 1 | Cov + Link + Exist, shared weights | 2.021143 | 1.594104 | 0.270800 |
| 1 | Cov + Link + Exist, branch-decoupled | 2.022070 | 1.593675 | 0.272600 |
| 1 | + structure-aware | 2.026121 | 1.596806 | 0.274600 |
| 1 | + EMA/floor | 2.252549 | 1.562365 | 0.525600 |
| 1 | + FID-FIA existence only | 2.204215 | 1.567794 | 0.459000 |
| 2 | Fixed Metropolis | 2.604818 | 1.581106 | 0.963600 |
| 2 | Cov only | 2.406650 | 1.522945 | 0.724000 |
| 2 | Link only | 2.034076 | 1.577558 | 0.305400 |
| 2 | Exist only | 2.481095 | 1.525096 | 0.805800 |
| 2 | Cov + Link | 2.025117 | 1.586352 | 0.282600 |
| 2 | Cov + Link + Exist, shared weights | 2.027101 | 1.586175 | 0.283600 |
| 2 | Cov + Link + Exist, branch-decoupled | 2.027779 | 1.586755 | 0.284800 |
| 2 | + structure-aware | 2.031797 | 1.589142 | 0.285200 |
| 2 | + EMA/floor | 2.231338 | 1.552068 | 0.506000 |
| 2 | + FID-FIA existence only | 2.182159 | 1.554998 | 0.444600 |
| 3 | Fixed Metropolis | 2.645651 | 1.595214 | 1.023600 |
| 3 | Cov only | 2.475933 | 1.531214 | 0.815400 |
| 3 | Link only | 2.065967 | 1.593461 | 0.335600 |
| 3 | Exist only | 2.535257 | 1.524611 | 0.882200 |
| 3 | Cov + Link | 2.055721 | 1.602648 | 0.311800 |
| 3 | Cov + Link + Exist, shared weights | 2.058193 | 1.603402 | 0.314000 |
| 3 | Cov + Link + Exist, branch-decoupled | 2.060716 | 1.602990 | 0.316800 |
| 3 | + structure-aware | 2.062937 | 1.606072 | 0.314800 |
| 3 | + EMA/floor | 2.295216 | 1.577301 | 0.572600 |
| 3 | + FID-FIA existence only | 2.234211 | 1.577844 | 0.502800 |
| 4 | Fixed Metropolis | 2.813069 | 1.665360 | 1.365400 |
| 4 | Cov only | 2.677570 | 1.578136 | 1.178400 |
| 4 | Link only | 2.146856 | 1.680046 | 0.387000 |
| 4 | Exist only | 2.764538 | 1.579840 | 1.304600 |
| 4 | Cov + Link | 2.137293 | 1.690050 | 0.359200 |
| 4 | Cov + Link + Exist, shared weights | 2.140099 | 1.693959 | 0.359600 |
| 4 | Cov + Link + Exist, branch-decoupled | 2.142087 | 1.693099 | 0.364200 |
| 4 | + structure-aware | 2.145814 | 1.696387 | 0.362400 |
| 4 | + EMA/floor | 2.395363 | 1.651781 | 0.675200 |
| 4 | + FID-FIA existence only | 2.333830 | 1.652948 | 0.579400 |
| 5 | Fixed Metropolis | 2.825234 | 1.681856 | 1.417200 |
| 5 | Cov only | 2.666391 | 1.617659 | 1.180600 |
| 5 | Link only | 2.094304 | 1.650850 | 0.287800 |
| 5 | Exist only | 2.772516 | 1.616576 | 1.358200 |
| 5 | Cov + Link | 2.085052 | 1.658244 | 0.270400 |
| 5 | Cov + Link + Exist, shared weights | 2.090301 | 1.660315 | 0.272000 |
| 5 | Cov + Link + Exist, branch-decoupled | 2.090633 | 1.658865 | 0.273400 |
| 5 | + structure-aware | 2.094262 | 1.661421 | 0.271800 |
| 5 | + EMA/floor | 2.319816 | 1.631839 | 0.573200 |
| 5 | + FID-FIA existence only | 2.271979 | 1.636347 | 0.490800 |
| 6 | Fixed Metropolis | 2.786321 | 1.644643 | 1.376600 |
| 6 | Cov only | 2.620629 | 1.589964 | 1.114400 |
| 6 | Link only | 2.078176 | 1.647797 | 0.257400 |
| 6 | Exist only | 2.738635 | 1.601146 | 1.341600 |
| 6 | Cov + Link | 2.069777 | 1.652674 | 0.243600 |
| 6 | Cov + Link + Exist, shared weights | 2.073158 | 1.653870 | 0.244600 |
| 6 | Cov + Link + Exist, branch-decoupled | 2.073457 | 1.653394 | 0.245400 |
| 6 | + structure-aware | 2.078309 | 1.657625 | 0.246000 |
| 6 | + EMA/floor | 2.298778 | 1.624324 | 0.531400 |
| 6 | + FID-FIA existence only | 2.261983 | 1.627771 | 0.470600 |
| 7 | Fixed Metropolis | 2.834022 | 1.637066 | 1.492800 |
| 7 | Cov only | 2.612210 | 1.600225 | 1.084000 |
| 7 | Link only | 2.083030 | 1.636510 | 0.286000 |
| 7 | Exist only | 2.693537 | 1.587224 | 1.220600 |
| 7 | Cov + Link | 2.070546 | 1.641159 | 0.265600 |
| 7 | Cov + Link + Exist, shared weights | 2.076002 | 1.642643 | 0.266800 |
| 7 | Cov + Link + Exist, branch-decoupled | 2.076434 | 1.641918 | 0.268000 |
| 7 | + structure-aware | 2.078666 | 1.644844 | 0.268400 |
| 7 | + EMA/floor | 2.303691 | 1.606751 | 0.553200 |
| 7 | + FID-FIA existence only | 2.253416 | 1.610207 | 0.486600 |
| 8 | Fixed Metropolis | 3.095421 | 1.632724 | 2.258200 |
| 8 | Cov only | 2.562676 | 1.576234 | 1.008000 |
| 8 | Link only | 2.058711 | 1.632091 | 0.250000 |
| 8 | Exist only | 2.659763 | 1.576207 | 1.165600 |
| 8 | Cov + Link | 2.050703 | 1.636085 | 0.237600 |
| 8 | Cov + Link + Exist, shared weights | 2.054890 | 1.637080 | 0.238800 |
| 8 | Cov + Link + Exist, branch-decoupled | 2.054585 | 1.636211 | 0.239600 |
| 8 | + structure-aware | 2.056173 | 1.637601 | 0.239600 |
| 8 | + EMA/floor | 2.247100 | 1.603881 | 0.470200 |
| 8 | + FID-FIA existence only | 2.215315 | 1.606447 | 0.434000 |
