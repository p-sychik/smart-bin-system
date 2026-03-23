"""Unit tests for mission queue and slot allocation logic."""

from mission_controller.mission_core import CollectionSlot
from mission_controller.mission_core import CollectionSlotManager
from mission_controller.mission_core import MissionQueue


def test_mission_queue_runs_jobs_in_fifo_order():
    queue = MissionQueue()
    job1 = queue.enqueue('BIN-1')
    job2 = queue.enqueue('BIN-2')
    job3 = queue.enqueue('BIN-3')

    assert queue.start_next() == job1
    assert queue.active_job == job1
    queue.complete_active()
    assert queue.start_next() == job2
    queue.complete_active()
    assert queue.start_next() == job3


def test_mission_queue_pause_blocks_next_start():
    queue = MissionQueue()
    queue.enqueue('BIN-1')
    queue.pause('operator intervention required')

    assert queue.start_next() is None
    assert queue.paused is True


def test_collection_slots_reserve_in_order_and_clear():
    manager = CollectionSlotManager(
        [
            CollectionSlot('slot_1', 0.45, 0.0, 0.0),
            CollectionSlot('slot_2', 0.45, 0.2, 0.0),
            CollectionSlot('slot_3', 0.45, -0.2, 0.0),
        ]
    )

    first = manager.reserve_next()
    manager.occupy(first)
    second = manager.reserve_next()
    manager.occupy(second)
    third = manager.reserve_next()

    assert first == 0
    assert second == 1
    assert third == 2

    manager.clear_all()
    assert [slot.state for slot in manager.slots] == ['FREE', 'FREE', 'FREE']
