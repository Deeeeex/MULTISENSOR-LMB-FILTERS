# V46 finite-episode weak-mixing audit

- Result SHA-256: `34a557ed6db987a542d576507bf614c6637add369685e73eabb032d61853f62d`
- MAT SHA-256: `cbebe8762e76f536ed407644fc3316397d0633adf6ea635a44230622c7c7a74d`
- Generation commit: `f0643b1bff9f670af6061a198ca63d5853bf3ae7` (clean: `1`)
- Cases: `32`; summary SHA-256: `abfb64fbad8c91e57f22a27d2b1740933c62d6b1e759f12975e6fa392e65de7d`

## Scope

The certificate is limited to the frozen 160-step episodes. It proves Boolean temporal influence propagation and finite-episode weak mixing at the registered theorem horizons. It does not prove infinite-horizon convergence, uniform-average consensus, tracking non-inferiority, or a validation claim.

The earliest audited horizon that gives full-positive support in every window of every case is `48` steps (M24: `32`, X36: `48`).

## Bias audit

Full-positive influence support does not imply uniform averaging. At H=`48`, the worst audited `||P-J||_2` is `0.908614`, the worst column-sum error is `0.411262`, and the worst influence L1 imbalance is `0.144533`.

At the registered theorem horizons, the worst `||P-J||_2` remains `0.639001` for M24 and `0.738417` for X36. The structural certificate therefore cannot replace the paired tracking non-inferiority test.

## Horizon aggregate

| H | all-support cases | all-positive-column cases | full-positive cases | scrambling windows | full-positive windows | numeric delta<1 cases | saturated delta windows | worst ||P-J||2 | worst column error |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4 | 0/32 | 0/32 | 0/32 | 0/5024 | 0/5024 | 0/32 | 0 | 1.00261 | 0.4 |
| 8 | 0/32 | 0/32 | 0/32 | 455/4896 | 0/4896 | 0/32 | 0 | 0.991324 | 0.38366 |
| 12 | 14/32 | 6/32 | 0/32 | 2870/4768 | 0/4768 | 14/32 | 0 | 0.982754 | 0.403867 |
| 16 | 28/32 | 24/32 | 0/32 | 4539/4640 | 488/4640 | 28/32 | 0 | 0.973545 | 0.395736 |
| 24 | 32/32 | 32/32 | 12/32 | 4384/4384 | 1836/4384 | 32/32 | 0 | 0.956628 | 0.405608 |
| 32 | 32/32 | 32/32 | 24/32 | 4128/4128 | 3120/4128 | 32/32 | 0 | 0.94054 | 0.408551 |
| 48 | 32/32 | 32/32 | 32/32 | 3616/3616 | 3616/3616 | 32/32 | 0 | 0.908614 | 0.411262 |
| 64 | 32/32 | 32/32 | 32/32 | 3104/3104 | 3104/3104 | 32/32 | 0 | 0.877432 | 0.380848 |
| 92 | 32/32 | 32/32 | 32/32 | 2208/2208 | 2208/2208 | 32/32 | 0 | 0.825273 | 0.381169 |
| 140 | 32/32 | 32/32 | 32/32 | 672/672 | 672/672 | 32/32 | 0 | 0.738417 | 0.268087 |

## Case audit

| preset | seed | N | repair pages | theorem H | earliest audited full-positive H | theorem pass | theorem worst delta | theorem worst ||P-J||2 |
|---|---:|---:|---:|---:|---:|:---:|---:|---:|
| m24-formation-fov | 41 | 24 | 0 | 92 | 24 | 1 | 0.24405 | 0.207128 |
| m24-formation-fov | 43 | 24 | 0 | 92 | 24 | 1 | 0.244094 | 0.207305 |
| m24-formation-fov | 47 | 24 | 0 | 92 | 24 | 1 | 0.243581 | 0.207302 |
| m24-formation-fov | 53 | 24 | 0 | 92 | 24 | 1 | 0.243693 | 0.206057 |
| m24-formation-fov-convoy | 41 | 24 | 0 | 92 | 32 | 1 | 0.768089 | 0.631245 |
| m24-formation-fov-convoy | 43 | 24 | 0 | 92 | 32 | 1 | 0.78214 | 0.638396 |
| m24-formation-fov-convoy | 47 | 24 | 0 | 92 | 32 | 1 | 0.779835 | 0.639001 |
| m24-formation-fov-convoy | 53 | 24 | 0 | 92 | 32 | 1 | 0.779835 | 0.639001 |
| m24-formation-fov-relay | 41 | 24 | 0 | 92 | 24 | 1 | 0.749689 | 0.61942 |
| m24-formation-fov-relay | 43 | 24 | 0 | 92 | 24 | 1 | 0.749689 | 0.61942 |
| m24-formation-fov-relay | 47 | 24 | 0 | 92 | 24 | 1 | 0.749689 | 0.61942 |
| m24-formation-fov-relay | 53 | 24 | 0 | 92 | 24 | 1 | 0.749689 | 0.61942 |
| m24-formation-fov-crossing | 41 | 24 | 0 | 92 | 32 | 1 | 0.772946 | 0.6348 |
| m24-formation-fov-crossing | 43 | 24 | 0 | 92 | 32 | 1 | 0.774986 | 0.635865 |
| m24-formation-fov-crossing | 47 | 24 | 0 | 92 | 32 | 1 | 0.77588 | 0.635886 |
| m24-formation-fov-crossing | 53 | 24 | 0 | 92 | 32 | 1 | 0.774918 | 0.635092 |
| x36-formation-fov | 41 | 36 | 0 | 140 | 24 | 1 | 0.412721 | 0.300441 |
| x36-formation-fov | 43 | 36 | 0 | 140 | 24 | 1 | 0.410631 | 0.298705 |
| x36-formation-fov | 47 | 36 | 0 | 140 | 24 | 1 | 0.411493 | 0.299761 |
| x36-formation-fov | 53 | 36 | 0 | 140 | 24 | 1 | 0.411123 | 0.299537 |
| x36-formation-fov-convoy | 41 | 36 | 0 | 140 | 48 | 1 | 0.881965 | 0.729813 |
| x36-formation-fov-convoy | 43 | 36 | 0 | 140 | 48 | 1 | 0.881239 | 0.727576 |
| x36-formation-fov-convoy | 47 | 36 | 0 | 140 | 48 | 1 | 0.881239 | 0.727576 |
| x36-formation-fov-convoy | 53 | 36 | 0 | 140 | 48 | 1 | 0.881239 | 0.727576 |
| x36-formation-fov-relay | 41 | 36 | 0 | 140 | 32 | 1 | 0.865368 | 0.715956 |
| x36-formation-fov-relay | 43 | 36 | 0 | 140 | 32 | 1 | 0.865368 | 0.715956 |
| x36-formation-fov-relay | 47 | 36 | 0 | 140 | 32 | 1 | 0.865368 | 0.715956 |
| x36-formation-fov-relay | 53 | 36 | 0 | 140 | 32 | 1 | 0.865368 | 0.715956 |
| x36-formation-fov-crossing | 41 | 36 | 3 | 140 | 48 | 1 | 0.888141 | 0.735588 |
| x36-formation-fov-crossing | 43 | 36 | 4 | 140 | 48 | 1 | 0.887931 | 0.737518 |
| x36-formation-fov-crossing | 47 | 36 | 3 | 140 | 48 | 1 | 0.888269 | 0.737888 |
| x36-formation-fov-crossing | 53 | 36 | 4 | 140 | 48 | 1 | 0.888592 | 0.738417 |
