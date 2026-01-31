from sqlalchemy import Column, Integer, String, Float, DateTime, Boolean, Text
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
from datetime import datetime

Base = declarative_base()

class Bin(Base):
    """Bin information model"""
    __tablename__ = "bins"
    
    id = Column(Integer, primary_key=True, index=True)
    bin_id = Column(String, unique=True, index=True, nullable=False)
    rfid_tag_id = Column(String, unique=True, index=True, nullable=False)
    bin_type = Column(String, nullable=False)  # e.g., "recycling", "waste", "compost"
    location_name = Column(String, nullable=False)
    path_to_bin_id = Column(String, nullable=False)  # Path ID to reach the bin
    path_to_collection_id = Column(String, nullable=False)  # Path ID from bin to collection point
    home_pose_x = Column(Float, default=0.0)
    home_pose_y = Column(Float, default=0.0)
    home_pose_z = Column(Float, default=0.0)
    home_pose_qx = Column(Float, default=0.0)
    home_pose_qy = Column(Float, default=0.0)
    home_pose_qz = Column(Float, default=0.0)
    home_pose_qw = Column(Float, default=1.0)
    last_collected = Column(DateTime, nullable=True)
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now())
    is_active = Column(Boolean, default=True)