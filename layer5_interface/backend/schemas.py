from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime

# Bin Schemas
class BinCreate(BaseModel):
    """Schema for creating a new bin"""
    rfid_tag_id: str = Field(..., description="Unique RFID tag ID")
    bin_type: str = Field(..., description="Type of bin (recycling, waste, compost)")
    location_name: str = Field(..., description="Human-readable location")
    path_to_bin_id: str = Field(..., description="Path ID to reach bin")
    path_to_collection_id: str = Field(..., description="Path ID from bin to collection")
    home_pose_x: float = 0.0
    home_pose_y: float = 0.0
    home_pose_z: float = 0.0
    home_pose_qx: float = 0.0
    home_pose_qy: float = 0.0
    home_pose_qz: float = 0.0
    home_pose_qw: float = 1.0
    
class BinUpdate(BaseModel):
    """Schema for updating bin information"""
    rfid_tag_id: Optional[str] = None
    bin_type: Optional[str] = None
    location_name: Optional[str] = None
    path_to_bin_id: Optional[str] = None
    path_to_collection_id: Optional[str] = None
    home_pose_x: Optional[float] = None
    home_pose_y: Optional[float] = None
    home_pose_z: Optional[float] = None
    home_pose_qx: Optional[float] = None
    home_pose_qy: Optional[float] = None
    home_pose_qz: Optional[float] = None
    home_pose_qw: Optional[float] = None
    is_active: Optional[bool] = None

class BinResponse(BaseModel):
    """Schema for bin response"""
    id: int
    bin_id: str
    rfid_tag_id: str
    bin_type: str
    location_name: str
    path_to_bin_id: str
    path_to_collection_id: str
    home_pose_x: float
    home_pose_y: float
    home_pose_z: float
    home_pose_qx: float
    home_pose_qy: float
    home_pose_qz: float
    home_pose_qw: float
    last_collected: Optional[datetime]
    created_at: datetime
    updated_at: datetime
    is_active: bool

    class Config:
        from_attributes = True
        
        
# Schedule Schemas
class ScheduleCreate(BaseModel):
    """Schema for creating a schedule"""
    bin_id: str = Field(..., description="Bin ID to collect")
    schedule_time: str = Field(..., description="Time in HH:MM format or cron")
    frequency: str = Field(..., description="daily, weekly, or custom")
    description: Optional[str] = None
    enabled: bool = True


class ScheduleUpdate(BaseModel):
    """Schema for updating schedule"""
    bin_id: Optional[str] = None
    schedule_time: Optional[str] = None
    frequency: Optional[str] = None
    description: Optional[str] = None
    enabled: Optional[bool] = None


class ScheduleResponse(BaseModel):
    """Schema for schedule response"""
    id: int
    schedule_id: str
    bin_id: str
    schedule_time: str
    frequency: str
    enabled: bool
    last_triggered: Optional[datetime]
    next_trigger: Optional[datetime]
    created_at: datetime
    updated_at: datetime
    description: Optional[str]

    class Config:
        from_attributes = True
