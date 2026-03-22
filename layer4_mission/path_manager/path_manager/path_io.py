"""Disk persistence for recorded paths."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List

from .path_core import PathPointRecord
from .path_core import points_from_dicts


@dataclass(frozen=True)
class StoredPath:
    """Path payload persisted on disk."""

    path_id: str
    description: str
    total_distance_m: float
    recorded_at_s: float
    points: List[PathPointRecord]


def resolve_storage_dir(storage_dir: str) -> Path:
    """Resolve and create the configured storage directory."""
    path = Path(storage_dir).expanduser()
    path.mkdir(parents=True, exist_ok=True)
    return path


def save_path(storage_dir: str, stored_path: StoredPath) -> Path:
    """Persist a path as JSON."""
    base_dir = resolve_storage_dir(storage_dir)
    path_file = base_dir / f'{stored_path.path_id}.json'
    payload = {
        'path_id': stored_path.path_id,
        'description': stored_path.description,
        'total_distance_m': stored_path.total_distance_m,
        'recorded_at_s': stored_path.recorded_at_s,
        'points': [
            {
                'x': point.x,
                'y': point.y,
                'theta': point.theta,
                'timestamp': point.timestamp,
            }
            for point in stored_path.points
        ],
    }
    path_file.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    return path_file


def load_path(storage_dir: str, path_id: str) -> StoredPath:
    """Load a path from JSON."""
    path_file = resolve_storage_dir(storage_dir) / f'{path_id}.json'
    payload = json.loads(path_file.read_text(encoding='utf-8'))
    return StoredPath(
        path_id=str(payload['path_id']),
        description=str(payload.get('description', '')),
        total_distance_m=float(payload.get('total_distance_m', 0.0)),
        recorded_at_s=float(payload.get('recorded_at_s', 0.0)),
        points=points_from_dicts(payload.get('points', [])),
    )
