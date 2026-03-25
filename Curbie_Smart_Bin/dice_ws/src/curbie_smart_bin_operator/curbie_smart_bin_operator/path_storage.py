"""JSON storage for multi-step operator paths."""

from __future__ import annotations

import json
import re
import time
from dataclasses import asdict
from dataclasses import dataclass
from dataclasses import field
from pathlib import Path
from typing import List

from .path_core import CommandRecord
from .path_core import commands_from_dicts
from .path_core import PathPointRecord
from .path_core import points_from_dicts


def sanitize_path_id(value: str) -> str:
    """Convert free-form path names into safe file identifiers."""
    text = (value or '').strip().lower()
    text = re.sub(r'[^a-z0-9._-]+', '-', text)
    text = re.sub(r'-{2,}', '-', text).strip('-')
    return text or 'path'


@dataclass(frozen=True)
class EventStep:
    """A non-motion recorded step."""

    event_type: str
    marker_id: int
    label: str = ''


@dataclass(frozen=True)
class MotionStep:
    """A recorded motion segment."""

    commands: List[CommandRecord] = field(default_factory=list)
    points: List[PathPointRecord] = field(default_factory=list)
    total_distance_m: float = 0.0
    total_duration_s: float = 0.0


@dataclass(frozen=True)
class RecordedStep:
    """A single recorded step in a stored operator path."""

    step_type: str
    motion: MotionStep | None = None
    event: EventStep | None = None


@dataclass(frozen=True)
class StoredOperatorPath:
    """Top-level path document stored on disk."""

    path_id: str
    display_name: str
    description: str
    created_at_s: float
    updated_at_s: float
    steps: List[RecordedStep] = field(default_factory=list)

    @property
    def motion_point_count(self) -> int:
        total = 0
        for step in self.steps:
            if not step.motion:
                continue
            if step.motion.commands:
                total += len(step.motion.commands)
            else:
                total += len(step.motion.points)
        return total


def resolve_storage_dir(storage_dir: str) -> Path:
    """Resolve and create the configured storage directory."""
    path = Path(storage_dir).expanduser()
    path.mkdir(parents=True, exist_ok=True)
    return path


def _point_to_dict(point: PathPointRecord) -> dict:
    return {
        'x': point.x,
        'y': point.y,
        'theta': point.theta,
        'timestamp': point.timestamp,
    }


def _command_to_dict(command: CommandRecord) -> dict:
    return {
        'linear_x': command.linear_x,
        'angular_z': command.angular_z,
        'duration_s': command.duration_s,
    }


def _step_to_dict(step: RecordedStep) -> dict:
    if step.step_type == 'motion' and step.motion is not None:
        return {
            'step_type': 'motion',
            'motion': {
                'commands': [
                    _command_to_dict(command) for command in step.motion.commands
                ],
                'total_distance_m': step.motion.total_distance_m,
                'total_duration_s': step.motion.total_duration_s,
                'points': [_point_to_dict(point) for point in step.motion.points],
            },
        }
    if step.step_type == 'event' and step.event is not None:
        return {
            'step_type': 'event',
            'event': asdict(step.event),
        }
    raise ValueError(f'Unsupported recorded step: {step!r}')


def save_path(storage_dir: str, stored_path: StoredOperatorPath) -> Path:
    """Persist a path as JSON."""
    base_dir = resolve_storage_dir(storage_dir)
    path_file = base_dir / f'{stored_path.path_id}.json'
    payload = {
        'path_id': stored_path.path_id,
        'display_name': stored_path.display_name,
        'description': stored_path.description,
        'created_at_s': stored_path.created_at_s,
        'updated_at_s': stored_path.updated_at_s,
        'steps': [_step_to_dict(step) for step in stored_path.steps],
    }
    path_file.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    return path_file


def load_path(storage_dir: str, path_id: str) -> StoredOperatorPath:
    """Load a path from JSON."""
    path_file = resolve_storage_dir(storage_dir) / f'{sanitize_path_id(path_id)}.json'
    payload = json.loads(path_file.read_text(encoding='utf-8'))
    return path_from_payload(payload)


def path_from_payload(payload: dict) -> StoredOperatorPath:
    """Build a typed path model from a JSON dictionary."""
    steps: List[RecordedStep] = []
    for raw_step in payload.get('steps', []):
        step_type = str(raw_step.get('step_type', ''))
        if step_type == 'motion':
            motion = raw_step.get('motion', {})
            steps.append(
                RecordedStep(
                    step_type='motion',
                    motion=MotionStep(
                        commands=commands_from_dicts(motion.get('commands', [])),
                        points=points_from_dicts(motion.get('points', [])),
                        total_distance_m=float(motion.get('total_distance_m', 0.0)),
                        total_duration_s=float(motion.get('total_duration_s', 0.0)),
                    ),
                )
            )
        elif step_type == 'event':
            raw_event = raw_step.get('event', {})
            steps.append(
                RecordedStep(
                    step_type='event',
                    event=EventStep(
                        event_type=str(raw_event.get('event_type', '')),
                        marker_id=int(raw_event.get('marker_id', 0)),
                        label=str(raw_event.get('label', '')),
                    ),
                )
            )

    return StoredOperatorPath(
        path_id=str(payload.get('path_id', 'path')),
        display_name=str(payload.get('display_name', payload.get('path_id', 'path'))),
        description=str(payload.get('description', '')),
        created_at_s=float(payload.get('created_at_s', 0.0)),
        updated_at_s=float(payload.get('updated_at_s', payload.get('created_at_s', 0.0))),
        steps=steps,
    )


def list_paths(storage_dir: str) -> List[StoredOperatorPath]:
    """Load all stored paths sorted by update time descending."""
    base_dir = resolve_storage_dir(storage_dir)
    paths: List[StoredOperatorPath] = []
    for path_file in sorted(base_dir.glob('*.json')):
        try:
            payload = json.loads(path_file.read_text(encoding='utf-8'))
            paths.append(path_from_payload(payload))
        except Exception:
            continue
    paths.sort(key=lambda item: item.updated_at_s, reverse=True)
    return paths


def rename_path(storage_dir: str, old_path_id: str, new_name: str) -> StoredOperatorPath:
    """Rename a stored path and move its file on disk."""
    old_id = sanitize_path_id(old_path_id)
    new_id = sanitize_path_id(new_name)
    stored = load_path(storage_dir, old_id)
    updated = StoredOperatorPath(
        path_id=new_id,
        display_name=new_name.strip() or new_id,
        description=stored.description,
        created_at_s=stored.created_at_s,
        updated_at_s=time.time(),
        steps=stored.steps,
    )
    base_dir = resolve_storage_dir(storage_dir)
    old_file = base_dir / f'{old_id}.json'
    new_file = base_dir / f'{new_id}.json'
    if new_file.exists() and new_file != old_file:
        raise FileExistsError(f'{new_id} already exists.')
    save_path(storage_dir, updated)
    if old_file.exists() and old_file != new_file:
        old_file.unlink()
    return updated


def delete_path(storage_dir: str, path_id: str) -> None:
    """Delete a stored path if it exists."""
    path_file = resolve_storage_dir(storage_dir) / f'{sanitize_path_id(path_id)}.json'
    if path_file.exists():
        path_file.unlink()
