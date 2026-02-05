"""
FastAPI Main Application
Provides REST API for bin collection system
"""
from fastapi import FastAPI, Depends, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select, delete
from typing import List, Optional
import uuid
from datetime import datetime
import logging
import json

from database import get_session, init_db  
from models import Bin, Schedule                     
from schemas import (
    BinCreate, BinUpdate, BinResponse,
    ScheduleCreate, ScheduleUpdate, ScheduleResponse
)
from scheduler import scheduler_service


# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title="Bin Collection System API",
    description="REST API for managing autonomous bin collection robot",
    version="1.0.0"
)

# Startup events
@app.on_event("startup")
async def startup_event():
    """Initialize database on startup"""
    logger.info("Starting up application...")
    
    # Initialize database
    await init_db()
    logger.info("Database initialized")
    
    logger.info("Application startup complete")
    
@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown"""
    logger.info("Shutting down application...")
    scheduler_service.shutdown()
    logger.info("Application shutdown complete")


# ============= BIN ENDPOINTS =============

@app.post("/api/bins", response_model=BinResponse, status_code=201)
async def create_bin(bin_data: BinCreate, db: AsyncSession = Depends(get_session)):
    """Create a new bin"""
    # Generate unique bin_id
    bin_id = f"BIN-{uuid.uuid4().hex[:8].upper()}"
    
    # Check if RFID tag already exists
    result = await db.execute(select(Bin).where(Bin.rfid_tag_id == bin_data.rfid_tag_id))
    existing = result.scalar_one_or_none()
    if existing:
        raise HTTPException(status_code=400, detail="RFID tag already registered")
    
    # Create new bin
    new_bin = Bin(
        bin_id=bin_id,
        rfid_tag_id=bin_data.rfid_tag_id,
        bin_type=bin_data.bin_type,
        location_name=bin_data.location_name,
        path_to_bin_id=bin_data.path_to_bin_id,
        path_to_collection_id=bin_data.path_to_collection_id,
        home_pose_x=bin_data.home_pose_x,
        home_pose_y=bin_data.home_pose_y,
        home_pose_z=bin_data.home_pose_z,
        home_pose_qx=bin_data.home_pose_qx,
        home_pose_qy=bin_data.home_pose_qy,
        home_pose_qz=bin_data.home_pose_qz,
        home_pose_qw=bin_data.home_pose_qw,
    )
    
    db.add(new_bin)
    await db.commit()
    await db.refresh(new_bin)
    
    logger.info(f"Created bin: {bin_id}")
    return new_bin


@app.get("/api/bins", response_model=List[BinResponse])
async def list_bins(
    skip: int = 0,
    limit: int = 100,
    active_only: bool = True,
    db: AsyncSession = Depends(get_session)
):
    """List all bins"""
    query = select(Bin)
    if active_only:
        query = query.where(Bin.is_active == True)
    query = query.offset(skip).limit(limit)
    
    result = await db.execute(query)
    bins = result.scalars().all()
    return bins


@app.get("/api/bins/{bin_id}", response_model=BinResponse)
async def get_bin(bin_id: str, db: AsyncSession = Depends(get_session)):
    """Get a specific bin"""
    result = await db.execute(select(Bin).where(Bin.bin_id == bin_id))
    bin_obj = result.scalar_one_or_none()
    
    if not bin_obj:
        raise HTTPException(status_code=404, detail="Bin not found")
    
    return bin_obj


@app.put("/api/bins/{bin_id}", response_model=BinResponse)
async def update_bin(
    bin_id: str,
    bin_update: BinUpdate,
    db: AsyncSession = Depends(get_session)
):
    """Update bin information"""
    result = await db.execute(select(Bin).where(Bin.bin_id == bin_id))
    bin_obj = result.scalar_one_or_none()
    
    if not bin_obj:
        raise HTTPException(status_code=404, detail="Bin not found")
    
    # Update fields
    update_data = bin_update.dict(exclude_unset=True)
    for field, value in update_data.items():
        setattr(bin_obj, field, value)
    
    await db.commit()
    await db.refresh(bin_obj)
    
    logger.info(f"Updated bin: {bin_id}")
    return bin_obj


@app.delete("/api/bins/{bin_id}", status_code=204)
async def delete_bin(bin_id: str, db: AsyncSession = Depends(get_session)):
    """Soft delete a bin"""
    result = await db.execute(select(Bin).where(Bin.bin_id == bin_id))
    bin_obj = result.scalar_one_or_none()
    
    if not bin_obj:
        raise HTTPException(status_code=404, detail="Bin not found")
    
    bin_obj.is_active = False
    await db.commit()
    
    logger.info(f"Deleted bin: {bin_id}")
    return None


# ============= SCHEDULE ENDPOINTS =============

@app.post("/api/schedules", response_model=ScheduleResponse, status_code=201)
async def create_schedule(
    schedule_data: ScheduleCreate,
    db: AsyncSession = Depends(get_session)
):
    """Create a new collection schedule"""
    # Verify bin exists
    result = await db.execute(select(Bin).where(Bin.bin_id == schedule_data.bin_id))
    bin_obj = result.scalar_one_or_none()
    if not bin_obj:
        raise HTTPException(status_code=404, detail="Bin not found")
    
    # Generate schedule ID
    schedule_id = f"SCH-{uuid.uuid4().hex[:8].upper()}"
    
    # Parse and validate schedule
    try:
        trigger = scheduler_service.parse_schedule_time(
            schedule_data.schedule_time,
            schedule_data.frequency
        )
        next_trigger = scheduler_service.calculate_next_trigger(trigger)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    
    # Create schedule in database
    new_schedule = Schedule(
        schedule_id=schedule_id,
        bin_id=schedule_data.bin_id,
        schedule_time=schedule_data.schedule_time,
        frequency=schedule_data.frequency,
        enabled=schedule_data.enabled,
        description=schedule_data.description,
        next_trigger=next_trigger
    )
    
    db.add(new_schedule)
    await db.commit()
    await db.refresh(new_schedule)
    
    # Add to scheduler if enabled
    if schedule_data.enabled:
        async def trigger_mission(bin_id: str, sched_id: str):
            """Callback for scheduled mission trigger"""
            await trigger_scheduled_mission(bin_id, sched_id, db)
        
        await scheduler_service.add_schedule(
            schedule_id,
            schedule_data.bin_id,
            schedule_data.schedule_time,
            schedule_data.frequency,
            trigger_mission
        )
    
    logger.info(f"Created schedule: {schedule_id}")
    return new_schedule


@app.get("/api/schedules", response_model=List[ScheduleResponse])
async def list_schedules(
    skip: int = 0,
    limit: int = 100,
    bin_id: Optional[str] = None,
    db: AsyncSession = Depends(get_session)
):
    """List all schedules"""
    query = select(Schedule)
    if bin_id:
        query = query.where(Schedule.bin_id == bin_id)
    query = query.offset(skip).limit(limit)
    
    result = await db.execute(query)
    schedules = result.scalars().all()
    return schedules


@app.get("/api/schedules/{schedule_id}", response_model=ScheduleResponse)
async def get_schedule(schedule_id: str, db: AsyncSession = Depends(get_session)):
    """Get a specific schedule"""
    result = await db.execute(select(Schedule).where(Schedule.schedule_id == schedule_id))
    schedule = result.scalar_one_or_none()
    
    if not schedule:
        raise HTTPException(status_code=404, detail="Schedule not found")
    
    return schedule


@app.put("/api/schedules/{schedule_id}", response_model=ScheduleResponse)
async def update_schedule(
    schedule_id: str,
    schedule_update: ScheduleUpdate,
    db: AsyncSession = Depends(get_session)
):
    """Update a schedule"""
    result = await db.execute(select(Schedule).where(Schedule.schedule_id == schedule_id))
    schedule = result.scalar_one_or_none()
    
    if not schedule:
        raise HTTPException(status_code=404, detail="Schedule not found")
    
    # Update fields
    update_data = schedule_update.dict(exclude_unset=True)
    
    # If schedule time or frequency changed, re-register with scheduler
    needs_reschedule = ('schedule_time' in update_data or 
                       'frequency' in update_data or 
                       'enabled' in update_data)
    
    for field, value in update_data.items():
        setattr(schedule, field, value)
    
    await db.commit()
    await db.refresh(schedule)
    
    # Reschedule if needed
    if needs_reschedule:
        await scheduler_service.remove_schedule(schedule_id)
        if schedule.enabled:
            async def trigger_mission(bin_id: str, sched_id: str):
                await trigger_scheduled_mission(bin_id, sched_id, db)
            
            await scheduler_service.add_schedule(
                schedule_id,
                schedule.bin_id,
                schedule.schedule_time,
                schedule.frequency,
                trigger_mission
            )
    
    logger.info(f"Updated schedule: {schedule_id}")
    return schedule


@app.delete("/api/schedules/{schedule_id}", status_code=204)
async def delete_schedule(schedule_id: str, db: AsyncSession = Depends(get_session)):
    """Delete a schedule"""
    result = await db.execute(select(Schedule).where(Schedule.schedule_id == schedule_id))
    schedule = result.scalar_one_or_none()
    
    if not schedule:
        raise HTTPException(status_code=404, detail="Schedule not found")
    
    # Remove from scheduler
    await scheduler_service.remove_schedule(schedule_id)
    
    # Delete from database
    await db.execute(delete(Schedule).where(Schedule.schedule_id == schedule_id))
    await db.commit()
    
    logger.info(f"Deleted schedule: {schedule_id}")
    return None