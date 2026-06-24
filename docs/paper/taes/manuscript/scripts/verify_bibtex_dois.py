#!/usr/bin/env python3
"""Verify that every BibTeX DOI resolves through doi.org.

The readiness checker already validates citation keys and DOI fields. This
script adds the stronger publication-hygiene check: every DOI in the manuscript
bibliography must be accepted by the public DOI resolver. The generated JSON
and Markdown files are evidence artifacts consumed by the submission-readiness
gate.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "generated"
BIB = ROOT / "references.bib"
REPORT_JSON = OUT / "bibtex_doi_verification.json"
REPORT_MD = OUT / "BIBTEX_DOI_VERIFICATION.md"

RESOLVER_PREFIX = "https://doi.org/api/handles/"
USER_AGENT = "TAES label-barycenter DOI verifier (mailto:author@example.com)"


@dataclass
class BibEntry:
    key: str
    doi: str | None


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def parse_bib_entries(bib: str) -> list[BibEntry]:
    entries: list[BibEntry] = []
    for raw_entry in re.split(r"\n(?=@\w+\s*{)", bib.strip()):
        if not raw_entry.strip():
            continue
        key_match = re.match(r"@\w+\s*{\s*([^,\s]+)", raw_entry)
        if not key_match:
            continue
        doi_match = re.search(r"\bdoi\s*=\s*[{\"]([^}\"]+)[}\"]", raw_entry, flags=re.IGNORECASE)
        entries.append(
            BibEntry(
                key=key_match.group(1),
                doi=doi_match.group(1).strip() if doi_match else None,
            )
        )
    return entries


def resolver_url(doi: str) -> str:
    return RESOLVER_PREFIX + urllib.parse.quote(doi, safe="/")


def query_doi(doi: str, timeout: float, retries: int) -> dict[str, object]:
    last_error = ""
    url = resolver_url(doi)
    for attempt in range(retries + 1):
        try:
            request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
            with urllib.request.urlopen(request, timeout=timeout) as response:
                payload = json.loads(response.read().decode("utf-8"))
            response_code = int(payload.get("responseCode", -1))
            return {
                "doi": doi,
                "url": url,
                "response_code": response_code,
                "resolved": response_code == 1,
                "error": "",
            }
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, ValueError) as exc:
            last_error = f"{type(exc).__name__}: {exc}"
            if attempt < retries:
                time.sleep(0.4 * (attempt + 1))
    return {
        "doi": doi,
        "url": url,
        "response_code": None,
        "resolved": False,
        "error": last_error,
    }


def load_cache() -> dict[str, object] | None:
    if not REPORT_JSON.exists():
        return None
    try:
        return json.loads(REPORT_JSON.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None


def cache_is_usable(cache: dict[str, object] | None, bib_sha: str, entry_count: int) -> bool:
    if not isinstance(cache, dict):
        return False
    return (
        cache.get("status") == "pass"
        and cache.get("bib_sha256") == bib_sha
        and int(cache.get("entry_count", -1)) == entry_count
        and int(cache.get("resolved_count", -1)) == entry_count
    )


def write_reports(payload: dict[str, object]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    REPORT_JSON.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    lines = [
        "# BibTeX DOI Verification\n\n",
        f"- Status: `{payload['status']}`\n",
        f"- BibTeX SHA-256: `{payload['bib_sha256']}`\n",
        f"- Entries checked: {payload['entry_count']}\n",
        f"- Resolved DOI entries: {payload['resolved_count']}\n",
        f"- Resolver: `{RESOLVER_PREFIX}{{doi}}`\n",
    ]
    if payload.get("used_cache"):
        lines.append("- Cache fallback: used a previous pass with the same BibTeX hash because the resolver was unavailable.\n")
    lines.extend(
        [
            "\n## Entries\n\n",
            "| Key | DOI | Resolver response | Status |\n",
            "| --- | --- | ---: | --- |\n",
        ]
    )
    for entry in payload.get("entries", []):
        if not isinstance(entry, dict):
            continue
        response_code = entry.get("response_code")
        response_text = "" if response_code is None else str(response_code)
        status = "resolved" if entry.get("resolved") else f"unresolved: {entry.get('error', '')}"
        lines.append(f"| `{entry.get('key', '')}` | `{entry.get('doi', '')}` | {response_text} | {status} |\n")
    missing = payload.get("missing_doi", [])
    if missing:
        lines.append("\n## Missing DOI Fields\n\n")
        for key in missing:
            lines.append(f"- `{key}`\n")
    REPORT_MD.write_text("".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--retries", type=int, default=1)
    parser.add_argument("--no-cache", action="store_true", help="Fail on resolver errors instead of using a same-hash cached pass.")
    args = parser.parse_args()

    bib = BIB.read_text(encoding="utf-8")
    bib_sha = sha256_text(bib)
    entries = parse_bib_entries(bib)
    missing = [entry.key for entry in entries if not entry.doi]

    checked_entries: list[dict[str, object]] = []
    network_errors: list[str] = []
    for entry in entries:
        if not entry.doi:
            continue
        result = query_doi(entry.doi, timeout=args.timeout, retries=args.retries)
        result["key"] = entry.key
        checked_entries.append(result)
        if result.get("error"):
            network_errors.append(f"{entry.key}: {result['error']}")
        time.sleep(0.05)

    resolved = [entry for entry in checked_entries if entry.get("resolved")]
    unresolved = [entry for entry in checked_entries if not entry.get("resolved")]
    status = "pass" if not missing and not unresolved and len(resolved) == len(entries) else "error"

    payload: dict[str, object] = {
        "status": status,
        "bib_sha256": bib_sha,
        "entry_count": len(entries),
        "resolved_count": len(resolved),
        "missing_doi": missing,
        "unresolved": unresolved,
        "network_errors": network_errors,
        "used_cache": False,
        "entries": checked_entries,
    }

    if status != "pass" and network_errors and not args.no_cache:
        cache = load_cache()
        if cache_is_usable(cache, bib_sha, len(entries)):
            cache_payload = dict(cache)
            cache_payload["used_cache"] = True
            cache_payload["cache_reason"] = "DOI resolver unavailable during this build; cached same-BibTeX pass reused."
            write_reports(cache_payload)
            return

    write_reports(payload)
    if status != "pass":
        for key in missing:
            print(f"missing DOI field: {key}", file=sys.stderr)
        for entry in unresolved:
            print(f"unresolved DOI: {entry.get('key')} {entry.get('doi')} {entry.get('error')}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
