# V247 temporal task-coupled formation braid

## Decision being tested

V244 treated a target cohort as task-coupled to a communication-tree cut when
its registered source and destination formations lay on opposite sides of that
cut.  That condition is necessary but not sufficient in a moving-formation
scene.  The realised nearest-visible owner can be a third formation, and the
actual ownership transition can occur long after the communication cut fails.

V247 replaces that metadata-only condition with a temporal visibility gate.
For every initial-tree edge which becomes physically unavailable, at least one
target cohort must satisfy all of the following:

1. its realised owner is on the registered source side during at least half of
   the 24 steps immediately before the first edge failure;
2. its realised owner is on the destination side during at least half of the
   interval from the first failure through the following 36 steps;
3. at least half of the cohort's targets make a source-side to destination-side
   owner transition while the edge is unavailable.

The gate uses target geometry and sensor visibility only.  It never reads a
tracking outcome.

## Scale-independent scene rule

Sensor trajectories, the 120-degree field of view, 300 m sensing range, 270 m
communication range and formation-braid platform geometry remain unchanged.
Target routes are now generated relative to the moving source and destination
formation centres.  Each cohort uses a smooth transition centred on the paired
overtake phase or the midpoint between neighbouring overtake phases.  A
deterministic geometry-only selector assigns one of two service lanes to each
cohort so that formation-centre and inter-cohort clearance are jointly
maximised.  The same forward offset, lane offset and transition width are used
for M24, X36 and X48.

## Evidence boundary

Passing V247 means the benchmark contains a real, temporally aligned
information-flow dependency rather than a nominal route label.  It authorises
a tracker comparison on the new scene.  It does not show that dynamic routing,
residual edges or a learned selector improves E-OSPA, RMSE, consistency or
communication.
