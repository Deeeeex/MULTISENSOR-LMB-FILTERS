# ICASSP 2027 upload files

- `manuscript.pdf`: current five-page submission manuscript.
- `manuscript_source.zip`: minimal editable source tree, with an internal SHA-256 manifest and isolated-build instructions.
- `SHA256SUMS`: hashes of the two upload artifacts.

Regenerate and verify all three from the repository root:

```bash
/Users/dex/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 \
  docs/icassp2027_paper/submission/build_submission_bundle.py --verify
```

The source archive excludes generated TeX intermediates, PNG previews, experiment outputs, and internal review documents. Do not upload the source ZIP unless the ICASSP submission system requests editable source; the conference's 2027 detailed instructions are not yet published.
