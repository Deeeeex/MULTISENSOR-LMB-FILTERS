# AA Balanced/Cardinality Validation

Generated at: 2026-06-22 01:26:16

Comparison order: Balanced AA -> Balanced spatial-KLA AA -> Cardinality spatial-KLA AA -> Tuned spatial-KLA AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 50
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51]
- lmbParallelUpdateMode: AA
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- targetFormationLifeSpan: 24
- existenceThreshold: 0.180000
- maximumNumberOfGmComponents: 3
- minimumTrajectoryLength: 10
- maximumNumberOfLbpIterations: 150
- lbpConvergenceTolerance: 0.0001
- aaStrictWeights: 0
- linkModel: fixed
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

## Arm Configs
### Balanced AA
- enabled: 1
- method: factorized
- aaSpatialFusionMode: mixture
- emaAlpha: 0.700
- minWeight: 0.050
- spatialEmaAlpha: 0.700
- existenceEmaAlpha: 0.700
- spatialMinWeight: 0.050
- existenceMinWeight: 0.050
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useDecoupledKla: 1
- useStructureAwareKla: 1
- useFidFiaExistence: 0
- aaStrictWeights: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- fidFiaExistenceStrength: 0.500
- fidFiaExistenceMinScore: 0.400
- existenceMinWeight: 0.050

### Balanced spatial-KLA AA
- enabled: 1
- method: factorized
- aaSpatialFusionMode: kla
- emaAlpha: 0.000
- minWeight: 0.000
- spatialEmaAlpha: 0.000
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.000
- existenceMinWeight: 0.000
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useDecoupledKla: 1
- useStructureAwareKla: 1
- useFidFiaExistence: 0
- aaStrictWeights: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- fidFiaExistenceStrength: 0.500
- fidFiaExistenceMinScore: 0.400
- existenceMinWeight: 0.000

### Cardinality spatial-KLA AA
- enabled: 1
- method: factorized
- aaSpatialFusionMode: kla
- emaAlpha: 0.000
- minWeight: 0.000
- spatialEmaAlpha: 0.000
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.000
- existenceMinWeight: 0.000
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useDecoupledKla: 1
- useStructureAwareKla: 1
- useFidFiaExistence: 1
- aaStrictWeights: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- fidFiaExistenceStrength: 4.000
- fidFiaExistenceMinScore: 0.000
- existenceMinWeight: 0.000

### Tuned spatial-KLA AA
- enabled: 1
- method: factorized
- aaSpatialFusionMode: kla
- emaAlpha: 0.000
- minWeight: 0.000
- spatialEmaAlpha: 0.000
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.000
- existenceMinWeight: 0.000
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useDecoupledKla: 1
- useStructureAwareKla: 1
- useFidFiaExistence: 0
- aaStrictWeights: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- spatialDecouplingStrength: 1.000
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.750
- existenceStructureStrength: 0.080
- fidFiaExistenceStrength: 0.500
- fidFiaExistenceMinScore: 0.400
- existenceMinWeight: 0.000

## Per-Trial pDropBySensor
- Trial 1: [0.5 0.2 0 0.1 0.1 0.5 0.1 0.1]
- Trial 2: [0.5 0 0.1 0.1 0.1 0.1 0.5 0.2]
- Trial 3: [0 0.1 0.5 0.1 0.5 0.2 0.1 0.1]
- Trial 4: [0 0.1 0.5 0.1 0.1 0.1 0.2 0.5]
- Trial 5: [0.1 0.1 0.1 0.5 0.2 0 0.1 0.5]
- Trial 6: [0.1 0.2 0.1 0.5 0.1 0.5 0.1 0]
- Trial 7: [0.1 0 0.5 0.1 0.1 0.1 0.2 0.5]
- Trial 8: [0 0.1 0.5 0.1 0.1 0.2 0.5 0.1]
- Trial 9: [0.5 0.1 0.1 0 0.1 0.5 0.2 0.1]
- Trial 10: [0.1 0.5 0 0.5 0.1 0.2 0.1 0.1]
- Trial 11: [0.1 0.5 0 0.1 0.1 0.1 0.5 0.2]
- Trial 12: [0.2 0 0.1 0.5 0.1 0.1 0.5 0.1]
- Trial 13: [0.1 0.1 0.5 0.2 0.1 0.1 0.5 0]
- Trial 14: [0.5 0.5 0 0.2 0.1 0.1 0.1 0.1]
- Trial 15: [0.5 0.5 0.1 0 0.2 0.1 0.1 0.1]
- Trial 16: [0.1 0.2 0.1 0.5 0 0.1 0.5 0.1]
- Trial 17: [0.1 0.5 0.1 0.2 0.1 0.5 0.1 0]
- Trial 18: [0.1 0.5 0.1 0.5 0.2 0.1 0.1 0]
- Trial 19: [0.1 0.1 0.1 0.1 0.5 0.5 0 0.2]
- Trial 20: [0.5 0.5 0 0.1 0.1 0.1 0.1 0.2]
- Trial 21: [0.5 0.2 0.5 0.1 0.1 0 0.1 0.1]
- Trial 22: [0.1 0.1 0.1 0.5 0 0.2 0.1 0.5]
- Trial 23: [0.5 0.2 0.1 0.1 0.1 0.1 0 0.5]
- Trial 24: [0.1 0.5 0.2 0 0.5 0.1 0.1 0.1]
- Trial 25: [0.1 0.2 0.1 0.5 0.5 0 0.1 0.1]
- Trial 26: [0.1 0.1 0.5 0.1 0 0.5 0.1 0.2]
- Trial 27: [0.1 0.5 0.1 0.1 0 0.5 0.1 0.2]
- Trial 28: [0 0.5 0.1 0.2 0.1 0.5 0.1 0.1]
- Trial 29: [0.1 0.1 0 0.1 0.5 0.1 0.2 0.5]
- Trial 30: [0.1 0.1 0.5 0 0.1 0.5 0.2 0.1]
- Trial 31: [0.1 0.5 0.5 0.1 0 0.2 0.1 0.1]
- Trial 32: [0.5 0.5 0.1 0.1 0.2 0.1 0 0.1]
- Trial 33: [0.1 0.1 0.1 0 0.5 0.1 0.2 0.5]
- Trial 34: [0.2 0.1 0.1 0 0.5 0.5 0.1 0.1]
- Trial 35: [0.2 0.1 0.5 0.1 0 0.5 0.1 0.1]
- Trial 36: [0.5 0 0.1 0.5 0.2 0.1 0.1 0.1]
- Trial 37: [0.2 0.5 0.1 0.1 0.1 0.5 0 0.1]
- Trial 38: [0.1 0.5 0 0.1 0.5 0.1 0.1 0.2]
- Trial 39: [0.2 0.5 0 0.1 0.1 0.1 0.1 0.5]
- Trial 40: [0.1 0.2 0.5 0 0.5 0.1 0.1 0.1]
- Trial 41: [0.1 0 0.5 0.1 0.1 0.2 0.1 0.5]
- Trial 42: [0.1 0.1 0.5 0.5 0.1 0.2 0 0.1]
- Trial 43: [0.2 0.1 0.5 0 0.1 0.1 0.1 0.5]
- Trial 44: [0.5 0.2 0 0.1 0.5 0.1 0.1 0.1]
- Trial 45: [0.1 0.1 0.2 0.5 0 0.5 0.1 0.1]
- Trial 46: [0.2 0.1 0.5 0.1 0.1 0.1 0 0.5]
- Trial 47: [0.2 0.1 0 0.1 0.1 0.5 0.5 0.1]
- Trial 48: [0.1 0.1 0.1 0.1 0 0.5 0.2 0.5]
- Trial 49: [0.1 0.5 0.1 0.2 0 0.1 0.5 0.1]
- Trial 50: [0.5 0.5 0.1 0.2 0.1 0.1 0.1 0]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 2 | Balanced AA | 3.392086 | 4.342744 | 0.122500 |
| 1 | 2 | Balanced spatial-KLA AA | 1.685695 | 1.475949 | 0.018750 |
| 1 | 2 | Cardinality spatial-KLA AA | 2.279836 | 6.395845 | 0.172500 |
| 1 | 2 | Tuned spatial-KLA AA | 1.682637 | 1.474567 | 0.018750 |
| 2 | 3 | Balanced AA | 3.406421 | 4.576159 | 0.105000 |
| 2 | 3 | Balanced spatial-KLA AA | 1.671618 | 1.438268 | 0.035000 |
| 2 | 3 | Cardinality spatial-KLA AA | 2.234411 | 5.629785 | 0.182500 |
| 2 | 3 | Tuned spatial-KLA AA | 1.667430 | 1.435498 | 0.035000 |
| 3 | 4 | Balanced AA | 3.410320 | 4.165611 | 0.112500 |
| 3 | 4 | Balanced spatial-KLA AA | 1.666557 | 1.447361 | 0.031250 |
| 3 | 4 | Cardinality spatial-KLA AA | 2.305920 | 7.239613 | 0.208750 |
| 3 | 4 | Tuned spatial-KLA AA | 1.665450 | 1.448322 | 0.031250 |
| 4 | 5 | Balanced AA | 3.445449 | 4.218685 | 0.106250 |
| 4 | 5 | Balanced spatial-KLA AA | 1.611372 | 1.308543 | 0.042500 |
| 4 | 5 | Cardinality spatial-KLA AA | 2.205488 | 8.192288 | 0.185000 |
| 4 | 5 | Tuned spatial-KLA AA | 1.603341 | 1.300640 | 0.042500 |
| 5 | 6 | Balanced AA | 3.396053 | 4.151797 | 0.118750 |
| 5 | 6 | Balanced spatial-KLA AA | 1.615253 | 1.461160 | 0.027500 |
| 5 | 6 | Cardinality spatial-KLA AA | 2.074807 | 3.946007 | 0.180000 |
| 5 | 6 | Tuned spatial-KLA AA | 1.610089 | 1.457096 | 0.027500 |
| 6 | 7 | Balanced AA | 3.506625 | 4.252262 | 0.120000 |
| 6 | 7 | Balanced spatial-KLA AA | 1.607385 | 1.535852 | 0.035000 |
| 6 | 7 | Cardinality spatial-KLA AA | 2.253362 | 4.542761 | 0.203750 |
| 6 | 7 | Tuned spatial-KLA AA | 1.598491 | 1.528223 | 0.035000 |
| 7 | 8 | Balanced AA | 3.368197 | 4.255841 | 0.126250 |
| 7 | 8 | Balanced spatial-KLA AA | 1.752850 | 1.520069 | 0.040000 |
| 7 | 8 | Cardinality spatial-KLA AA | 2.426644 | 9.058411 | 0.242500 |
| 7 | 8 | Tuned spatial-KLA AA | 1.747386 | 1.519075 | 0.038750 |
| 8 | 9 | Balanced AA | 3.426704 | 4.309608 | 0.111250 |
| 8 | 9 | Balanced spatial-KLA AA | 1.617754 | 1.365570 | 0.028750 |
| 8 | 9 | Cardinality spatial-KLA AA | 2.247552 | 6.476970 | 0.185000 |
| 8 | 9 | Tuned spatial-KLA AA | 1.623301 | 1.359868 | 0.030000 |
| 9 | 10 | Balanced AA | 3.356317 | 3.889968 | 0.116250 |
| 9 | 10 | Balanced spatial-KLA AA | 1.666117 | 1.534908 | 0.051250 |
| 9 | 10 | Cardinality spatial-KLA AA | 2.398329 | 9.427002 | 0.231250 |
| 9 | 10 | Tuned spatial-KLA AA | 1.664538 | 1.534739 | 0.051250 |
| 10 | 11 | Balanced AA | 3.368787 | 3.956830 | 0.145000 |
| 10 | 11 | Balanced spatial-KLA AA | 1.761833 | 1.508536 | 0.038750 |
| 10 | 11 | Cardinality spatial-KLA AA | 2.289048 | 3.759117 | 0.195000 |
| 10 | 11 | Tuned spatial-KLA AA | 1.768428 | 1.504540 | 0.040000 |
| 11 | 12 | Balanced AA | 3.271158 | 3.921300 | 0.108750 |
| 11 | 12 | Balanced spatial-KLA AA | 1.675307 | 1.468030 | 0.036250 |
| 11 | 12 | Cardinality spatial-KLA AA | 2.438882 | 8.624778 | 0.233750 |
| 11 | 12 | Tuned spatial-KLA AA | 1.673244 | 1.464816 | 0.036250 |
| 12 | 13 | Balanced AA | 3.498404 | 4.192866 | 0.138750 |
| 12 | 13 | Balanced spatial-KLA AA | 1.812651 | 1.645526 | 0.051250 |
| 12 | 13 | Cardinality spatial-KLA AA | 2.364393 | 5.124741 | 0.230000 |
| 12 | 13 | Tuned spatial-KLA AA | 1.809269 | 1.614177 | 0.051250 |
| 13 | 14 | Balanced AA | 3.482350 | 4.430543 | 0.120000 |
| 13 | 14 | Balanced spatial-KLA AA | 1.685225 | 1.561644 | 0.023750 |
| 13 | 14 | Cardinality spatial-KLA AA | 2.478281 | 8.115482 | 0.236250 |
| 13 | 14 | Tuned spatial-KLA AA | 1.671022 | 1.560164 | 0.022500 |
| 14 | 15 | Balanced AA | 3.385448 | 4.043115 | 0.107500 |
| 14 | 15 | Balanced spatial-KLA AA | 1.726311 | 1.454982 | 0.038750 |
| 14 | 15 | Cardinality spatial-KLA AA | 2.366225 | 5.301943 | 0.206250 |
| 14 | 15 | Tuned spatial-KLA AA | 1.720003 | 1.450386 | 0.038750 |
| 15 | 16 | Balanced AA | 3.382170 | 4.087418 | 0.097500 |
| 15 | 16 | Balanced spatial-KLA AA | 1.646908 | 1.563859 | 0.022500 |
| 15 | 16 | Cardinality spatial-KLA AA | 2.269800 | 6.641728 | 0.170000 |
| 15 | 16 | Tuned spatial-KLA AA | 1.641038 | 1.559038 | 0.022500 |
| 16 | 17 | Balanced AA | 3.406813 | 4.070152 | 0.155000 |
| 16 | 17 | Balanced spatial-KLA AA | 1.696564 | 1.399230 | 0.046250 |
| 16 | 17 | Cardinality spatial-KLA AA | 2.178782 | 5.749259 | 0.153750 |
| 16 | 17 | Tuned spatial-KLA AA | 1.685404 | 1.388587 | 0.046250 |
| 17 | 18 | Balanced AA | 3.439743 | 4.396258 | 0.101250 |
| 17 | 18 | Balanced spatial-KLA AA | 1.661538 | 1.467810 | 0.020000 |
| 17 | 18 | Cardinality spatial-KLA AA | 2.327320 | 8.478921 | 0.188750 |
| 17 | 18 | Tuned spatial-KLA AA | 1.654544 | 1.461745 | 0.020000 |
| 18 | 19 | Balanced AA | 3.435933 | 4.146228 | 0.127500 |
| 18 | 19 | Balanced spatial-KLA AA | 1.677983 | 1.472265 | 0.030000 |
| 18 | 19 | Cardinality spatial-KLA AA | 2.334281 | 5.324394 | 0.201250 |
| 18 | 19 | Tuned spatial-KLA AA | 1.676684 | 1.472405 | 0.030000 |
| 19 | 20 | Balanced AA | 3.295977 | 4.271184 | 0.091250 |
| 19 | 20 | Balanced spatial-KLA AA | 1.712894 | 1.671799 | 0.037500 |
| 19 | 20 | Cardinality spatial-KLA AA | 2.264509 | 6.184752 | 0.210000 |
| 19 | 20 | Tuned spatial-KLA AA | 1.709979 | 1.671988 | 0.037500 |
| 20 | 21 | Balanced AA | 3.565287 | 4.561847 | 0.107500 |
| 20 | 21 | Balanced spatial-KLA AA | 1.738831 | 1.492037 | 0.042500 |
| 20 | 21 | Cardinality spatial-KLA AA | 2.423272 | 8.289152 | 0.231250 |
| 20 | 21 | Tuned spatial-KLA AA | 1.734930 | 1.489287 | 0.042500 |
| 21 | 22 | Balanced AA | 3.435532 | 4.182614 | 0.158750 |
| 21 | 22 | Balanced spatial-KLA AA | 1.643992 | 1.437069 | 0.030000 |
| 21 | 22 | Cardinality spatial-KLA AA | 2.555277 | 9.744355 | 0.285000 |
| 21 | 22 | Tuned spatial-KLA AA | 1.636799 | 1.428640 | 0.030000 |
| 22 | 23 | Balanced AA | 3.326913 | 3.783781 | 0.131250 |
| 22 | 23 | Balanced spatial-KLA AA | 1.778702 | 1.491748 | 0.043750 |
| 22 | 23 | Cardinality spatial-KLA AA | 2.384849 | 8.209044 | 0.190000 |
| 22 | 23 | Tuned spatial-KLA AA | 1.780009 | 1.492509 | 0.043750 |
| 23 | 24 | Balanced AA | 3.468044 | 4.196340 | 0.116250 |
| 23 | 24 | Balanced spatial-KLA AA | 1.705914 | 1.508775 | 0.025000 |
| 23 | 24 | Cardinality spatial-KLA AA | 2.311440 | 5.076860 | 0.212500 |
| 23 | 24 | Tuned spatial-KLA AA | 1.702572 | 1.505093 | 0.026250 |
| 24 | 25 | Balanced AA | 3.396059 | 4.283356 | 0.085000 |
| 24 | 25 | Balanced spatial-KLA AA | 1.751865 | 1.506417 | 0.041250 |
| 24 | 25 | Cardinality spatial-KLA AA | 2.399482 | 6.423909 | 0.218750 |
| 24 | 25 | Tuned spatial-KLA AA | 1.749069 | 1.504870 | 0.041250 |
| 25 | 26 | Balanced AA | 3.363214 | 4.255944 | 0.122500 |
| 25 | 26 | Balanced spatial-KLA AA | 1.775951 | 1.501875 | 0.066250 |
| 25 | 26 | Cardinality spatial-KLA AA | 2.480973 | 8.321548 | 0.253750 |
| 25 | 26 | Tuned spatial-KLA AA | 1.776746 | 1.504229 | 0.066250 |
| 26 | 27 | Balanced AA | 3.498225 | 4.167946 | 0.150000 |
| 26 | 27 | Balanced spatial-KLA AA | 1.696856 | 1.402655 | 0.042500 |
| 26 | 27 | Cardinality spatial-KLA AA | 2.272023 | 5.373128 | 0.176250 |
| 26 | 27 | Tuned spatial-KLA AA | 1.690300 | 1.398991 | 0.041250 |
| 27 | 28 | Balanced AA | 3.420165 | 4.292542 | 0.106250 |
| 27 | 28 | Balanced spatial-KLA AA | 1.786148 | 1.625140 | 0.031250 |
| 27 | 28 | Cardinality spatial-KLA AA | 2.399638 | 5.799768 | 0.195000 |
| 27 | 28 | Tuned spatial-KLA AA | 1.785713 | 1.624857 | 0.031250 |
| 28 | 29 | Balanced AA | 3.410505 | 4.196787 | 0.128750 |
| 28 | 29 | Balanced spatial-KLA AA | 1.730800 | 1.418444 | 0.060000 |
| 28 | 29 | Cardinality spatial-KLA AA | 2.249612 | 7.053279 | 0.186250 |
| 28 | 29 | Tuned spatial-KLA AA | 1.726645 | 1.416342 | 0.060000 |
| 29 | 30 | Balanced AA | 3.343666 | 4.071159 | 0.152500 |
| 29 | 30 | Balanced spatial-KLA AA | 1.652775 | 1.411792 | 0.033750 |
| 29 | 30 | Cardinality spatial-KLA AA | 2.478688 | 8.936778 | 0.277500 |
| 29 | 30 | Tuned spatial-KLA AA | 1.647364 | 1.406921 | 0.033750 |
| 30 | 31 | Balanced AA | 3.429518 | 4.181148 | 0.122500 |
| 30 | 31 | Balanced spatial-KLA AA | 1.690827 | 1.425912 | 0.048750 |
| 30 | 31 | Cardinality spatial-KLA AA | 2.306549 | 5.643404 | 0.192500 |
| 30 | 31 | Tuned spatial-KLA AA | 1.688300 | 1.424875 | 0.048750 |
| 31 | 32 | Balanced AA | 3.458313 | 4.336210 | 0.117500 |
| 31 | 32 | Balanced spatial-KLA AA | 1.742021 | 1.525576 | 0.021250 |
| 31 | 32 | Cardinality spatial-KLA AA | 2.230526 | 5.044853 | 0.166250 |
| 31 | 32 | Tuned spatial-KLA AA | 1.743321 | 1.528194 | 0.021250 |
| 32 | 33 | Balanced AA | 3.427634 | 4.127693 | 0.107500 |
| 32 | 33 | Balanced spatial-KLA AA | 1.621318 | 1.402106 | 0.023750 |
| 32 | 33 | Cardinality spatial-KLA AA | 2.216062 | 4.300132 | 0.190000 |
| 32 | 33 | Tuned spatial-KLA AA | 1.613191 | 1.396453 | 0.023750 |
| 33 | 34 | Balanced AA | 3.419265 | 4.292562 | 0.125000 |
| 33 | 34 | Balanced spatial-KLA AA | 1.696558 | 1.747630 | 0.025000 |
| 33 | 34 | Cardinality spatial-KLA AA | 2.367889 | 4.815178 | 0.220000 |
| 33 | 34 | Tuned spatial-KLA AA | 1.695320 | 1.748860 | 0.025000 |
| 34 | 35 | Balanced AA | 3.334593 | 4.249668 | 0.107500 |
| 34 | 35 | Balanced spatial-KLA AA | 1.618175 | 1.507592 | 0.026250 |
| 34 | 35 | Cardinality spatial-KLA AA | 2.332046 | 6.436983 | 0.238750 |
| 34 | 35 | Tuned spatial-KLA AA | 1.610147 | 1.512261 | 0.025000 |
| 35 | 36 | Balanced AA | 3.470570 | 4.186904 | 0.090000 |
| 35 | 36 | Balanced spatial-KLA AA | 1.622899 | 1.427197 | 0.021250 |
| 35 | 36 | Cardinality spatial-KLA AA | 2.321843 | 5.720165 | 0.205000 |
| 35 | 36 | Tuned spatial-KLA AA | 1.616460 | 1.423190 | 0.021250 |
| 36 | 37 | Balanced AA | 3.421605 | 4.337009 | 0.130000 |
| 36 | 37 | Balanced spatial-KLA AA | 1.782242 | 1.527651 | 0.031250 |
| 36 | 37 | Cardinality spatial-KLA AA | 2.381234 | 7.068227 | 0.215000 |
| 36 | 37 | Tuned spatial-KLA AA | 1.780506 | 1.528165 | 0.031250 |
| 37 | 38 | Balanced AA | 3.485310 | 4.499052 | 0.096250 |
| 37 | 38 | Balanced spatial-KLA AA | 1.667723 | 1.446154 | 0.038750 |
| 37 | 38 | Cardinality spatial-KLA AA | 2.361615 | 7.625978 | 0.201250 |
| 37 | 38 | Tuned spatial-KLA AA | 1.660233 | 1.444823 | 0.037500 |
| 38 | 39 | Balanced AA | 3.479446 | 4.097254 | 0.121250 |
| 38 | 39 | Balanced spatial-KLA AA | 1.614356 | 1.516545 | 0.021250 |
| 38 | 39 | Cardinality spatial-KLA AA | 2.322913 | 6.143893 | 0.216250 |
| 38 | 39 | Tuned spatial-KLA AA | 1.611358 | 1.515457 | 0.022500 |
| 39 | 40 | Balanced AA | 3.410947 | 4.250717 | 0.131250 |
| 39 | 40 | Balanced spatial-KLA AA | 1.664036 | 1.372591 | 0.035000 |
| 39 | 40 | Cardinality spatial-KLA AA | 2.158771 | 3.746949 | 0.180000 |
| 39 | 40 | Tuned spatial-KLA AA | 1.667447 | 1.373777 | 0.036250 |
| 40 | 41 | Balanced AA | 3.336184 | 4.018987 | 0.115000 |
| 40 | 41 | Balanced spatial-KLA AA | 1.628021 | 1.399461 | 0.021250 |
| 40 | 41 | Cardinality spatial-KLA AA | 2.248325 | 6.190125 | 0.188750 |
| 40 | 41 | Tuned spatial-KLA AA | 1.625285 | 1.397315 | 0.021250 |
| 41 | 42 | Balanced AA | 3.428635 | 4.216232 | 0.102500 |
| 41 | 42 | Balanced spatial-KLA AA | 1.605840 | 1.440623 | 0.018750 |
| 41 | 42 | Cardinality spatial-KLA AA | 2.389492 | 8.790858 | 0.232500 |
| 41 | 42 | Tuned spatial-KLA AA | 1.598724 | 1.434675 | 0.017500 |
| 42 | 43 | Balanced AA | 3.430078 | 4.478616 | 0.088750 |
| 42 | 43 | Balanced spatial-KLA AA | 1.632382 | 1.342508 | 0.037500 |
| 42 | 43 | Cardinality spatial-KLA AA | 2.243872 | 4.207570 | 0.228750 |
| 42 | 43 | Tuned spatial-KLA AA | 1.622933 | 1.336654 | 0.036250 |
| 43 | 44 | Balanced AA | 3.479449 | 4.392078 | 0.083750 |
| 43 | 44 | Balanced spatial-KLA AA | 1.634557 | 1.403110 | 0.048750 |
| 43 | 44 | Cardinality spatial-KLA AA | 2.229659 | 4.603942 | 0.210000 |
| 43 | 44 | Tuned spatial-KLA AA | 1.627533 | 1.397986 | 0.048750 |
| 44 | 45 | Balanced AA | 3.409840 | 4.571214 | 0.112500 |
| 44 | 45 | Balanced spatial-KLA AA | 1.729610 | 1.470438 | 0.040000 |
| 44 | 45 | Cardinality spatial-KLA AA | 2.385090 | 6.895152 | 0.210000 |
| 44 | 45 | Tuned spatial-KLA AA | 1.726464 | 1.468741 | 0.040000 |
| 45 | 46 | Balanced AA | 3.434247 | 4.548666 | 0.110000 |
| 45 | 46 | Balanced spatial-KLA AA | 1.652361 | 1.403927 | 0.045000 |
| 45 | 46 | Cardinality spatial-KLA AA | 2.457978 | 9.569102 | 0.225000 |
| 45 | 46 | Tuned spatial-KLA AA | 1.650691 | 1.398505 | 0.046250 |
| 46 | 47 | Balanced AA | 3.561738 | 4.413306 | 0.126250 |
| 46 | 47 | Balanced spatial-KLA AA | 1.672641 | 1.444772 | 0.031250 |
| 46 | 47 | Cardinality spatial-KLA AA | 2.448574 | 5.927669 | 0.247500 |
| 46 | 47 | Tuned spatial-KLA AA | 1.670539 | 1.445587 | 0.031250 |
| 47 | 48 | Balanced AA | 3.441057 | 4.221793 | 0.133750 |
| 47 | 48 | Balanced spatial-KLA AA | 1.735532 | 1.505959 | 0.032500 |
| 47 | 48 | Cardinality spatial-KLA AA | 2.442521 | 7.128036 | 0.217500 |
| 47 | 48 | Tuned spatial-KLA AA | 1.729929 | 1.502969 | 0.032500 |
| 48 | 49 | Balanced AA | 3.290493 | 3.854972 | 0.090000 |
| 48 | 49 | Balanced spatial-KLA AA | 1.647913 | 1.399372 | 0.031250 |
| 48 | 49 | Cardinality spatial-KLA AA | 2.354440 | 6.304673 | 0.251250 |
| 48 | 49 | Tuned spatial-KLA AA | 1.643617 | 1.395513 | 0.031250 |
| 49 | 50 | Balanced AA | 3.596770 | 4.681684 | 0.113750 |
| 49 | 50 | Balanced spatial-KLA AA | 1.789049 | 1.530109 | 0.070000 |
| 49 | 50 | Cardinality spatial-KLA AA | 2.338485 | 7.628604 | 0.195000 |
| 49 | 50 | Tuned spatial-KLA AA | 1.781736 | 1.523159 | 0.070000 |
| 50 | 51 | Balanced AA | 3.475646 | 4.271333 | 0.123750 |
| 50 | 51 | Balanced spatial-KLA AA | 1.669040 | 1.471470 | 0.028750 |
| 50 | 51 | Cardinality spatial-KLA AA | 2.258570 | 6.250594 | 0.188750 |
| 50 | 51 | Tuned spatial-KLA AA | 1.664178 | 1.467082 | 0.028750 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Balanced AA | 3.422478 | 4.237960 | 0.116775 |
| Balanced spatial-KLA AA | 1.686615 | 1.476160 | 0.035375 |
| Cardinality spatial-KLA AA | 2.329792 | 6.549674 | 0.209250 |
| Tuned spatial-KLA AA | 1.682607 | 1.472837 | 0.035350 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Balanced AA | OSPA | 3.422478 +/- 0.067020 | [3.403901, 3.441055] | 50 |
| Balanced spatial-KLA AA | OSPA | 1.686615 +/- 0.056643 | [1.670914, 1.702316] | 50 |
| Cardinality spatial-KLA AA | OSPA | 2.329792 +/- 0.096160 | [2.303137, 2.356446] | 50 |
| Tuned spatial-KLA AA | OSPA | 1.682607 +/- 0.058035 | [1.666520, 1.698693] | 50 |
| Balanced AA | Loc. disag. | 4.237960 +/- 0.191917 | [4.184763, 4.291156] | 50 |
| Balanced spatial-KLA AA | Loc. disag. | 1.476160 +/- 0.082159 | [1.453387, 1.498934] | 50 |
| Cardinality spatial-KLA AA | Loc. disag. | 6.549674 +/- 1.627397 | [6.098583, 7.000766] | 50 |
| Tuned spatial-KLA AA | Loc. disag. | 1.472837 +/- 0.082422 | [1.449991, 1.495683] | 50 |
| Balanced AA | Card. disp. | 0.116775 +/- 0.017823 | [0.111835, 0.121715] | 50 |
| Balanced spatial-KLA AA | Card. disp. | 0.035375 +/- 0.011781 | [0.032109, 0.038641] | 50 |
| Cardinality spatial-KLA AA | Card. disp. | 0.209250 +/- 0.027834 | [0.201535, 0.216965] | 50 |
| Tuned spatial-KLA AA | Card. disp. | 0.035350 +/- 0.011802 | [0.032079, 0.038621] | 50 |

## Paired Improvements Relative to Balanced AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Balanced spatial-KLA AA | OSPA | 1.735863 +/- 0.085376 | [1.712198, 1.759528] | 50.72% | 50/50 | 1.776e-15 |
| Cardinality spatial-KLA AA | OSPA | 1.092687 +/- 0.117710 | [1.060059, 1.125314] | 31.93% | 50/50 | 1.776e-15 |
| Tuned spatial-KLA AA | OSPA | 1.739871 +/- 0.087085 | [1.715733, 1.764010] | 50.84% | 50/50 | 1.776e-15 |
| Balanced spatial-KLA AA | Loc. disag. | 2.761799 +/- 0.206144 | [2.704659, 2.818940] | 65.17% | 50/50 | 1.776e-15 |
| Cardinality spatial-KLA AA | Loc. disag. | -2.311714 +/- 1.631857 | [-2.764042, -1.859387] | -54.55% | 4/50 | 4.462e-10 |
| Tuned spatial-KLA AA | Loc. disag. | 2.765123 +/- 0.206169 | [2.707975, 2.822270] | 65.25% | 50/50 | 1.776e-15 |
| Balanced spatial-KLA AA | Card. disp. | 0.081400 +/- 0.020069 | [0.075837, 0.086963] | 69.71% | 50/50 | 1.776e-15 |
| Cardinality spatial-KLA AA | Card. disp. | -0.092475 +/- 0.032088 | [-0.101369, -0.083581] | -79.19% | 1/50 | 9.059e-14 |
| Tuned spatial-KLA AA | Card. disp. | 0.081425 +/- 0.020005 | [0.075880, 0.086970] | 69.73% | 50/50 | 1.776e-15 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Balanced AA | 3.641417 | 4.396212 | 0.221925 |
| Balanced spatial-KLA AA | 2.024428 | 3.678611 | 0.087925 |
| Cardinality spatial-KLA AA | 2.442527 | 5.542318 | 0.314250 |
| Tuned spatial-KLA AA | 2.029641 | 3.682880 | 0.088000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Balanced AA | E-OSPA | 3.641417 +/- 0.059666 | [3.624878, 3.657955] | 50 |
| Balanced spatial-KLA AA | E-OSPA | 2.024428 +/- 0.060615 | [2.007626, 2.041229] | 50 |
| Cardinality spatial-KLA AA | E-OSPA | 2.442527 +/- 0.079261 | [2.420557, 2.464497] | 50 |
| Tuned spatial-KLA AA | E-OSPA | 2.029641 +/- 0.061460 | [2.012605, 2.046677] | 50 |
| Balanced AA | RMSE | 4.396212 +/- 0.323660 | [4.306498, 4.485926] | 50 |
| Balanced spatial-KLA AA | RMSE | 3.678611 +/- 0.309247 | [3.592892, 3.764330] | 50 |
| Cardinality spatial-KLA AA | RMSE | 5.542318 +/- 0.756993 | [5.332491, 5.752146] | 50 |
| Tuned spatial-KLA AA | RMSE | 3.682880 +/- 0.308425 | [3.597389, 3.768371] | 50 |
| Balanced AA | CardErr | 0.221925 +/- 0.029964 | [0.213619, 0.230231] | 50 |
| Balanced spatial-KLA AA | CardErr | 0.087925 +/- 0.015604 | [0.083600, 0.092250] | 50 |
| Cardinality spatial-KLA AA | CardErr | 0.314250 +/- 0.046808 | [0.301276, 0.327224] | 50 |
| Tuned spatial-KLA AA | CardErr | 0.088000 +/- 0.015584 | [0.083680, 0.092320] | 50 |

## Paired Local-Metric Improvements Relative to Balanced AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Balanced spatial-KLA AA | E-OSPA | 1.616989 +/- 0.068479 | [1.598008, 1.635970] | 44.41% | 50/50 | 1.776e-15 |
| Cardinality spatial-KLA AA | E-OSPA | 1.198890 +/- 0.098913 | [1.171473, 1.226307] | 32.92% | 50/50 | 1.776e-15 |
| Tuned spatial-KLA AA | E-OSPA | 1.611776 +/- 0.069394 | [1.592541, 1.631011] | 44.26% | 50/50 | 1.776e-15 |
| Balanced spatial-KLA AA | RMSE | 0.717601 +/- 0.357521 | [0.618502, 0.816701] | 16.32% | 49/50 | 9.059e-14 |
| Cardinality spatial-KLA AA | RMSE | -1.146106 +/- 0.834662 | [-1.377463, -0.914749] | -26.07% | 2/50 | 2.267e-12 |
| Tuned spatial-KLA AA | RMSE | 0.713332 +/- 0.359198 | [0.613768, 0.812897] | 16.23% | 49/50 | 9.059e-14 |
| Balanced spatial-KLA AA | CardErr | 0.134000 +/- 0.027476 | [0.126384, 0.141616] | 60.38% | 50/50 | 1.776e-15 |
| Cardinality spatial-KLA AA | CardErr | -0.092325 +/- 0.054695 | [-0.107486, -0.077164] | -41.60% | 2/50 | 2.267e-12 |
| Tuned spatial-KLA AA | CardErr | 0.133925 +/- 0.027317 | [0.126353, 0.141497] | 60.35% | 50/50 | 1.776e-15 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Balanced AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Balanced AA | 57.738269 +/- 6.647526 | 2.405761 | 1.000x | 50 |
| Balanced spatial-KLA AA | 49.301794 +/- 3.528084 | 2.054241 | 0.858x | 50 |
| Cardinality spatial-KLA AA | 64.439389 +/- 7.793919 | 2.684975 | 1.121x | 50 |
| Tuned spatial-KLA AA | 49.891611 +/- 6.631214 | 2.078817 | 0.868x | 50 |
