"""
Scheduler Service - Manages automatic bin collection schedules
"""
from apscheduler.schedulers.asyncio import AsyncIOScheduler
from apscheduler.triggers.cron import CronTrigger
from datetime import datetime
from typing import Optional
import logging

logger = logging.getLogger(__name__)

class SchedulerService:
    """Manages scheduled bin collections"""
    
    def __init__(self):
        self.scheduler = AsyncIOScheduler()
        self.scheduler.start()
        self.jobs = {}  # schedule_id -> job_id mapping
        logger.info("Scheduler service started")
    
    def parse_schedule_time(self, schedule_time: str, frequency: str):
        """
        Parse schedule time and frequency into APScheduler trigger
        
        Args:
            schedule_time: Time string (e.g., "14:30", "*/5 * * * *")
            frequency: "daily", "weekly", or "custom"
        
        Returns:
            APScheduler trigger
        """
        if frequency == "daily":
            # Parse HH:MM format
            try:
                hour, minute = map(int, schedule_time.split(':'))
                return CronTrigger(hour=hour, minute=minute)
            except ValueError:
                raise ValueError(f"Invalid time format for daily: {schedule_time}. Use HH:MM")
        
        elif frequency == "weekly":
            # Parse "DAY HH:MM" format (e.g., "monday 14:30")
            try:
                parts = schedule_time.lower().split()
                if len(parts) != 2:
                    raise ValueError("Weekly format should be 'DAY HH:MM'")
                
                day_of_week = parts[0]
                hour, minute = map(int, parts[1].split(':'))
                
                # Convert day name to number (0=Monday, 6=Sunday)
                days = {
                    'monday': 0, 'tuesday': 1, 'wednesday': 2, 'thursday': 3,
                    'friday': 4, 'saturday': 5, 'sunday': 6
                }
                
                if day_of_week not in days:
                    raise ValueError(f"Invalid day: {day_of_week}")
                
                return CronTrigger(day_of_week=days[day_of_week], hour=hour, minute=minute)
            except Exception as e:
                raise ValueError(f"Invalid weekly format: {schedule_time}. Use 'DAY HH:MM'. Error: {e}")
        
        elif frequency == "custom":
            # Assume cron format for custom
            try:
                parts = schedule_time.split()
                if len(parts) == 5:
                    minute, hour, day, month, day_of_week = parts
                    return CronTrigger(
                        minute=minute,
                        hour=hour,
                        day=day,
                        month=month,
                        day_of_week=day_of_week
                    )
                else:
                    raise ValueError("Custom cron format should have 5 fields")
            except Exception as e:
                raise ValueError(f"Invalid cron format: {schedule_time}. Error: {e}")
        
        else:
            raise ValueError(f"Unknown frequency: {frequency}")
    
    def calculate_next_trigger(self, trigger) -> Optional[datetime]:
        """Calculate next trigger time"""
        try:
            next_fire_time = trigger.get_next_fire_time(None, datetime.now())
            return next_fire_time
        except Exception as e:
            logger.error(f"Failed to calculate next trigger: {e}")
            return None
    
    async def add_schedule(self, schedule_id: str, bin_id: str, schedule_time: str, 
                          frequency: str, trigger_callback):
        """
        Add a new schedule
        
        Args:
            schedule_id: Unique schedule identifier
            bin_id: Bin to collect
            schedule_time: Time specification
            frequency: Schedule frequency
            trigger_callback: Async function to call when triggered
        """
        try:
            trigger = self.parse_schedule_time(schedule_time, frequency)
            
            job = self.scheduler.add_job(
                trigger_callback,
                trigger,
                args=[bin_id, schedule_id],
                id=schedule_id,
                replace_existing=True,
                misfire_grace_time=300  # 5 minutes grace period
            )
            
            self.jobs[schedule_id] = job.id
            logger.info(f"Added schedule {schedule_id} for bin {bin_id}")
            
            return trigger
            
        except Exception as e:
            logger.error(f"Failed to add schedule: {e}")
            raise
    
    async def remove_schedule(self, schedule_id: str):
        """Remove a schedule"""
        try:
            if schedule_id in self.jobs:
                self.scheduler.remove_job(self.jobs[schedule_id])
                del self.jobs[schedule_id]
                logger.info(f"Removed schedule {schedule_id}")
        except Exception as e:
            logger.error(f"Failed to remove schedule: {e}")
            raise
    
    async def pause_schedule(self, schedule_id: str):
        """Pause a schedule"""
        try:
            if schedule_id in self.jobs:
                self.scheduler.pause_job(self.jobs[schedule_id])
                logger.info(f"Paused schedule {schedule_id}")
        except Exception as e:
            logger.error(f"Failed to pause schedule: {e}")
            raise
    
    async def resume_schedule(self, schedule_id: str):
        """Resume a paused schedule"""
        try:
            if schedule_id in self.jobs:
                self.scheduler.resume_job(self.jobs[schedule_id])
                logger.info(f"Resumed schedule {schedule_id}")
        except Exception as e:
            logger.error(f"Failed to resume schedule: {e}")
            raise
    
    def get_schedule_info(self, schedule_id: str):
        """Get information about a schedule"""
        try:
            if schedule_id in self.jobs:
                job = self.scheduler.get_job(self.jobs[schedule_id])
                if job:
                    return {
                        "id": job.id,
                        "next_run_time": job.next_run_time.isoformat() if job.next_run_time else None,
                        "trigger": str(job.trigger)
                    }
        except Exception as e:
            logger.error(f"Failed to get schedule info: {e}")
        return None
    
    def shutdown(self):
        """Shutdown the scheduler"""
        self.scheduler.shutdown()
        logger.info("Scheduler service shutdown")


# Global scheduler instance
scheduler_service = SchedulerService()