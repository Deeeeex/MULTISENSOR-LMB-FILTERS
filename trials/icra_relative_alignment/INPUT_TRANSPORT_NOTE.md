# Public raw-input transport retry

The first raw-feature preparation process exited with code 1 after a TLS
handshake timeout while initializing an anonymous public Box download.
Its thread pool finished other already scheduled input jobs, leaving valid
CRC-checked compact shards. No corrected tracking run had started.

`prepare_features_retry.py` adds bounded retries to that public connection
setup only. The registered parser, geometry, occupancy, packet, translation,
native filter, comparison gates and raw source inventory are unchanged.
`resume_features.py` reuses verified shards through the original preparer.
It archives each failed execution receipt byte-for-byte and retains every
attempt's complete log. `FEATURE_EXECUTION.json` is the latest execution
journal state, with explicit links to its previous attempts.

The original first-attempt log is `RUN/ICRA_RELATIVE_ALIGNMENT/features.log`.
No credentials, ephemeral public-read tokens or signed URLs are persisted.
