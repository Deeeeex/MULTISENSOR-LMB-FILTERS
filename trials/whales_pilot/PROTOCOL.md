# WHALES 2-D cooperative-fusion pilot

Authorized by the user on 2026-09-07; work belongs to `codex/icra`.
This is an independent exploratory experiment, not an ICASSP table update.

## Frozen selection and applicability checks

- Source: official WHALES `whales_meta.tar.zst`, Drive file
  `17G5RAihDVfj05IG6Nsq9Kpdu8X205D6e` (207096707 bytes).
- Inspect validation metadata only. Choose the lexicographically first scene
  with at least nine moving-agent slots and at least 80 synchronized frames.
  This selects `2024-02-26-07-10-14` (10 vehicle slots, one RSU, 99 frames).
- Use vehicle slots 0--8, excluding the RSU. Keep all 99 frames (0.5 s spacing).
  Exclude all original sensing vehicles from the target set, including slot 9.
  Use the non-sensing object annotations with persistent scene-local indices.
- Form three fixed groups of three by minimizing the sum of initial squared
  within-group pair distances; break ties lexicographically. No future
  trajectories or tracking outcomes enter the grouping.
- Use a 100 m distance-limited physical communication graph as an explicit
  pilot assumption, not a measured WHALES radio property. Do not increase this
  range or regroup using future trajectories after observing feasibility.
- Check identity continuity, synchronization, finite coordinates, and run the
  unchanged V240/V242 policy functions over the actual geometry. A failure of
  the fixed-group architecture stops the four-arm tracking experiment; retain
  its timestamp and cause. Do not fabricate edges or substitute another policy.

## Tracking stage, conditional on applicability

Four arms: local-only LMB, fixed formation tree, full causal repair, sparse V242.
Three observation/packet seeds: 2701, 2702, 2703. Pair the measurements and
directed delivery uniforms between arms. No parameter search.
Position observation noise 1 m, nominal detection probability 0.9,
Poisson clutter mean 1 per node/frame, 120 degree heading-aligned FoV,
60 m sensing range, independent directed packet drop probability 0.1,
one fusion round per frame, OSPA order 2 and cutoff 30 m.
These are synthetic observation/radio assumptions, not recorded detections.
Birth priors must be specified separately from realized target trajectories;
the current truth-initialized scenario constructor must not be used unchanged.

Report set error, count error, conditional position RMSE and its valid support,
node disagreement/tails, posterior bytes, and runtime. Three random seeds of
one trajectory do not establish cross-scene generalization. If applicability
fails, report no tracking gain and do not launch the twelve filter runs.

## Sources

- https://github.com/chensiweiTHU/WHALES
- https://arxiv.org/html/2411.13340v2
- https://drive.google.com/drive/folders/1L9xkBAfFox1WZR5aBGhmW6jF9dXIBLru
- Official `tools/data_converter/whales.py` and
  `tools/misc/visualize_whales.py` define the annotation and world-pose fields.

Large downloads remain under ignored `tmp/whales/`. Keep compact converted
input, source hashes, geometry results and figures with this pilot.
