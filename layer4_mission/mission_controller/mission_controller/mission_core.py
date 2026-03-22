"""Pure queue and slot helpers for the mission controller."""

from __future__ import annotations

import uuid
from dataclasses import dataclass
from typing import List, Optional


@dataclass(frozen=True)
class MissionJob:
    """Queued mission request."""

    mission_id: str
    bin_id: str


@dataclass
class CollectionSlot:
    """Configured collection slot state."""

    name: str
    stand_off_m: float
    lateral_offset_m: float
    yaw_offset_rad: float
    state: str = 'FREE'


class MissionQueue:
    """FIFO queue with a hard one-bin active lock."""

    def __init__(self) -> None:
        self.pending: List[MissionJob] = []
        self.active_job: Optional[MissionJob] = None
        self.paused_reason: str = ''

    @property
    def paused(self) -> bool:
        return bool(self.paused_reason)

    def enqueue(self, bin_id: str) -> MissionJob:
        job = MissionJob(
            mission_id=f'MIS-{uuid.uuid4().hex[:8].upper()}',
            bin_id=bin_id,
        )
        self.pending.append(job)
        return job

    def start_next(self) -> Optional[MissionJob]:
        if self.active_job is not None or self.paused or not self.pending:
            return None
        self.active_job = self.pending.pop(0)
        return self.active_job

    def complete_active(self) -> Optional[MissionJob]:
        finished = self.active_job
        self.active_job = None
        return finished

    def pause(self, reason: str) -> None:
        self.paused_reason = reason

    def clear_pause(self) -> None:
        self.paused_reason = ''

    def abort_all(self) -> None:
        self.pending = []
        self.active_job = None
        self.paused_reason = ''


class CollectionSlotManager:
    """Reserve and occupy collection slots in a predictable order."""

    def __init__(self, slots: List[CollectionSlot]) -> None:
        self.slots = slots

    def reserve_next(self) -> Optional[int]:
        for index, slot in enumerate(self.slots):
            if slot.state == 'FREE':
                slot.state = 'RESERVED'
                return index
        return None

    def release(self, index: Optional[int]) -> None:
        if index is None or index < 0 or index >= len(self.slots):
            return
        if self.slots[index].state == 'RESERVED':
            self.slots[index].state = 'FREE'

    def occupy(self, index: Optional[int]) -> None:
        if index is None or index < 0 or index >= len(self.slots):
            return
        self.slots[index].state = 'OCCUPIED'

    def clear_all(self) -> None:
        for slot in self.slots:
            slot.state = 'FREE'
