# Formation-backbone M24 source-input freeze v1

This registry freezes only deterministic inputs for the fixed-reference
source-posterior run. It authorizes no candidate continuation, paired
tracking, ground-truth scoring, X36 claim, or validation claim. The public
filter state-estimate output is suppressed for every registered source
permit; only the predecision cache and its provenance may be published.

## Provenance

- Method source commit: `5de0aa61a271b6999e8723af3b7378d8c7fd2aa0`
- Executable source commit: `0b6f2d4565463803897db73bdc93b19b1c4a0e23`
- Executable source manifest SHA-256: `6d4017abf6c8d3baf86596fa0ee21520cf6a2aee60357e0127e11d54bfe3b702`
- Critical-path resolution SHA-256: `eb1a0a01fb0d18434982c15815f9441a28c508bb53ebc6c1bc6f19b1c16da0a0`
- Freeze-pending protocol SHA-256 used for discovery: `12da780f33a40b8df9a6964eec3f2c4e6145f14078cdcbb1dea9ff309bdb67e3`
- Discovery manifest SHA-256: `890c02cd1145914d28dea9cfb8a229673f7f4c08e3b2dc8d2a03ec5b6e4da688`
- Discovery MAT file SHA-256: `a6d864ec10a694e920a1ff765ad048da7cf43005d05f82f94cbd053fba110c71`
- Frozen case-set SHA-256: `68607c9b55b9ec25e4bb0c845d0a1ceec8f2c9f8902efce1d8248bd81988dee3`
- Frozen protocol SHA-256: `02a6bc2951621eb22a56d8763d28bd922bcc0fbb4323b958856d1da05f78c310`

The discovery manifest was independently rehashed before registration. All
15 input fingerprints reproduced their component hashes, were unique, and
matched the final protocol registry exactly in scene-major, seed-ascending
order.

## Registered cases

| # | Case | Input SHA-256 | Case SHA-256 | Permit SHA-256 |
|--:|:--|:--|:--|:--|
| 1 | `m24-formation-fov__seed41__w55-135` | `7bf6889be4ed0036ee3fc2c4997107dfec171b186648c9499038d234d44ed18b` | `5d86045620f76b5144cfc8fa12e33ae436ecab62756b5a71c20d1a7bdba14da8` | `5184e4a976cd32ec0b92d146476248d12e31c9368516596b6e2a6b8d5e8db94c` |
| 2 | `m24-formation-fov__seed43__w55-135` | `555db702f9cbb32409be7e21cba15c8364cfe8e032e66dcb93025c5f97f0c18d` | `80d67c8144af8936ca8d62627dec4271f1184a06d1b21f60c27ed1bb9536c1a7` | `085359cb2b90b0be98fa61e139eaf9592469c37280c9fd966aefc096e6fcdda0` |
| 3 | `m24-formation-fov__seed47__w55-135` | `0fd469f59ce5a1c4c6f897589e6dc6b6f217a89ea1d6ae600bc8fdef53ecf5b4` | `916b3a604805655a5687440201bf003cd6c8da98e4fdccd51d4dae59e16654ee` | `5ebffc79dde744dcf8b944fb6df138ec891ea3daec94072bd1c322f88ccec159` |
| 4 | `m24-formation-fov__seed53__w55-135` | `08bc1e9c17d8d206471cb9bb6a005d9ae72f3286c9b4ebd9a490e8525a9d3451` | `47c2bc4d3908152eba58ed790765164bc90a4347bdc2bf5a7fb7a8e8586e3c90` | `9855990ccd1953981c59a1cf3b87b42f1ee137b5c4d5ea352e926e3a52dccff5` |
| 5 | `m24-formation-fov__seed59__w55-135` | `ca2520cbbab4df9b55debf3780beb3468344f8cb94ae21e93efb4ee5de2f298c` | `868c42a701bda30166f1de67d7a7e876572c7a11ceeea8e29b641ee9c802a566` | `fe5aba6db6ea34aa10857e056a4a4edfa6b4a4628a7cb508c56f683b719a7905` |
| 6 | `m24-formation-fov-convoy__seed41__w45-140` | `02feda120ca55e839ea12ab9694b6dbb1bb64c277e9af3e98fadf6481b75bd84` | `775816228705acae67399dea7ae78066682547a566cabbbb0a191836746fd6ad` | `7ebe65492cffd3da251a05272c07591a34736032a676d5edf1b275d35ca26236` |
| 7 | `m24-formation-fov-convoy__seed43__w45-140` | `92febdecf7fc183d381dc33cd6bbdcf06c4ded6a25f030c4eaff8f359678cb36` | `a2f4ceec85b6402b1c83dbfa69b3f1527b025be6cd0d034b01ca3ce6a49ff589` | `0959c999da6226ff6cfd426c8ff492f8658d8f399596419e018ad02211ec97c1` |
| 8 | `m24-formation-fov-convoy__seed47__w45-140` | `7177e6054d51bccbdd5cca9285bb4d1042d599370efc9b5e906030471cb53d07` | `cfe057fd0954e03d7977f3e5901fdd2817fd453a56519fa63221bce6f50f9f45` | `bb87286d8d4078a308981c536985f5b6525583cecc618c03c2f7e235f62cc124` |
| 9 | `m24-formation-fov-convoy__seed53__w45-140` | `75d15dc2e721e1214866905f862329f2102c109e889b73f292daa3a85dbf3596` | `8f786e605b84aa09c37631f2e3e69f9ee5db266757cb74892e06fa6649475eae` | `1add7294d1a90a7603419b14270bedb12d23a24b5d3a22fc6d7c3921900942e0` |
| 10 | `m24-formation-fov-convoy__seed59__w45-140` | `3b00b300731af752ec718f0e5c85a06d0ef782f61968c7ba044df15fb4f81dce` | `2b5b25b38e9d73f85c8826a96f1878e8011b45a02e13689c40b82352b3fe73a4` | `3c88454a2cd416c82c563e29cc37254e0f6acb2067b8913ad7d3cf2dbec7c65d` |
| 11 | `m24-formation-fov-relay__seed41__w45-140` | `7a85608a634f5ba4b772da1b6bb24bdcdfc5224b2a769ff7631c1f1a3ef6f2ef` | `75d8bed1cd68b744b7bb018db7d978d15444025fceef50bdcde7361b56d21261` | `2947088a45d2d656abb1f1ce281c20f2eceb20312127fe212fe0a62797f9f663` |
| 12 | `m24-formation-fov-relay__seed43__w45-140` | `2d533669a476cbe293ee18782d3da1d8bfc609e92ab780213cecd03d90cfc905` | `5c871a6a2635b666a7553581561f40ba304039f43b8215aa902976e67bf6a847` | `fe1d9e7470948cebfe92314113c70a60fb6dd5b8a9731b783cb702ff52b85f1c` |
| 13 | `m24-formation-fov-relay__seed47__w45-140` | `f3942319c055faba74bb1d4aff7bb923faf84f8b6b5da46bde62b89734bbcd62` | `75754b280ef68f90210490dedab27409d136d6ff6e5f6e754a1768bed4b96ab0` | `132470a44ebe75568b5b88ea12be0889090c0a3e26b19eebc34f78470d2391ec` |
| 14 | `m24-formation-fov-relay__seed53__w45-140` | `c6f9649790320e0926ea9f0386a90318ad0acb869edc782e398b71b71a9b6813` | `e0709308b103dcc25583b5edd8296618903694b6b9b0bee854dff69ba5a99911` | `34b87e9d183e60e7addb9dd540f3d932181d511ee9e7e166ed415d367cb63765` |
| 15 | `m24-formation-fov-relay__seed59__w45-140` | `82da0798814d4073cd8edab8d4380e3c30decd6f093ae1a14afad369266a4da1` | `f2b044ab33deb8e6fdc67407d2dbb85be369d90a7cf97a8f4f2042304ec3e026` | `f7513dc3127aa0136cef1c42dbd7cafc1e0641194ccc0f96b8e51c28c56b922a` |

## Still sealed

- Candidate continuation: false
- Paired tracking: false
- State-estimate output: false
- Tracking-outcome scoring: false
- Ground-truth access: false
- Future-outcome access: false
- Validation claims: false
