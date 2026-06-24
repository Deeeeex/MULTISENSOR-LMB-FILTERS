"""Shared source paths for manuscript evidence extraction and verification."""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[3]
SOURCE_MANIFEST = ROOT / "evidence_sources.json"


def load_sources() -> dict[str, str]:
    if not SOURCE_MANIFEST.exists():
        raise FileNotFoundError(f"Missing evidence source manifest: {SOURCE_MANIFEST}")
    return json.loads(SOURCE_MANIFEST.read_text(encoding="utf-8"))


def evidence_path(key: str) -> Path:
    sources = load_sources()
    if key not in sources:
        raise KeyError(f"Missing evidence source key `{key}` in {SOURCE_MANIFEST}")
    path = REPO / sources[key]
    if not path.exists():
        raise FileNotFoundError(f"Evidence source `{key}` does not exist: {path}")
    return path


def optional_evidence_path(key: str) -> Path | None:
    sources = load_sources()
    if key not in sources:
        return None
    path = REPO / sources[key]
    if not path.exists():
        raise FileNotFoundError(f"Evidence source `{key}` does not exist: {path}")
    return path
