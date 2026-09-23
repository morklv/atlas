"""Small local SQLite experiment ledger for repeatable ATLAS runs."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sqlite3
from typing import Any


SCHEMA = """
CREATE TABLE IF NOT EXISTS experiments (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    created_at TEXT NOT NULL,
    kind TEXT NOT NULL,
    input_name TEXT NOT NULL,
    input_sha256 TEXT NOT NULL,
    configuration_json TEXT NOT NULL,
    metrics_json TEXT NOT NULL,
    artifacts_json TEXT NOT NULL
);
"""


def file_sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def record_experiment(
    database: str | Path,
    *,
    kind: str,
    input_path: str | Path,
    configuration: dict[str, Any],
    metrics: dict[str, Any],
    artifacts: dict[str, Any],
) -> int:
    """Add one immutable run record and return its local identifier."""
    if not kind.strip():
        raise ValueError("kind must not be empty")
    input_path = Path(input_path)
    database = Path(database)
    database.parent.mkdir(parents=True, exist_ok=True)
    with sqlite3.connect(database) as connection:
        connection.execute(SCHEMA)
        cursor = connection.execute(
            """INSERT INTO experiments
               (created_at, kind, input_name, input_sha256, configuration_json, metrics_json, artifacts_json)
               VALUES (?, ?, ?, ?, ?, ?, ?)""",
            (
                datetime.now(timezone.utc).isoformat(),
                kind,
                input_path.name,
                file_sha256(input_path),
                json.dumps(configuration, sort_keys=True),
                json.dumps(metrics, sort_keys=True),
                json.dumps(artifacts, sort_keys=True),
            ),
        )
        return int(cursor.lastrowid)


def recent_experiments(database: str | Path, limit: int = 20) -> list[dict[str, Any]]:
    """Read latest records without exposing absolute local input paths."""
    if limit < 1 or limit > 100:
        raise ValueError("limit must be between 1 and 100")
    database = Path(database)
    if not database.is_file():
        return []
    with sqlite3.connect(database) as connection:
        connection.execute(SCHEMA)
        rows = connection.execute(
            """SELECT id, created_at, kind, input_name, input_sha256,
                      configuration_json, metrics_json, artifacts_json
                 FROM experiments ORDER BY id DESC LIMIT ?""",
            (limit,),
        ).fetchall()
    keys = ("id", "created_at", "kind", "input_name", "input_sha256", "configuration", "metrics", "artifacts")
    output = []
    for row in rows:
        value = dict(zip(keys, row, strict=True))
        for field in ("configuration", "metrics", "artifacts"):
            value[field] = json.loads(value[field])
        output.append(value)
    return output
