"""Measured-terrain fusion used by the ATLAS simulation and future ROS 2 node."""
from dataclasses import dataclass
import numpy as np

ROAD, BUILDING, FOREST, LOW_VEGETATION, WATER = 1, 2, 3, 4, 5

@dataclass(frozen=True)
class FusionMap:
    elevation: np.ndarray
    slope_deg: np.ndarray
    traversability: np.ndarray
    confidence: np.ndarray
    blocked: np.ndarray

def fuse(labels, lidar_height, observed, resolution_m):
    """Fuse semantic labels with measured heights without inventing unseen geometry."""
    labels=np.asarray(labels,dtype=np.uint8); height=np.asarray(lidar_height,dtype=float); observed=np.asarray(observed,dtype=bool)
    if labels.shape!=height.shape or labels.shape!=observed.shape or labels.ndim!=2: raise ValueError('labels, height, and observed must share a 2D shape')
    if not np.isfinite(resolution_m) or resolution_m<=0: raise ValueError('resolution_m must be positive')
    measured=observed & np.isfinite(height)
    fill=np.where(measured,height,0.)
    gy,gx=np.gradient(fill,resolution_m); slope=np.degrees(np.arctan(np.hypot(gx,gy)))
    blocked=(labels==BUILDING)|(labels==FOREST)|(labels==WATER)|(~measured)|(slope>28)
    terrain_penalty=np.select([labels==ROAD,labels==LOW_VEGETATION],[0.,.55],default=.25)
    traversability=np.clip(1-terrain_penalty-slope/55,0,1); traversability[blocked]=0
    confidence=np.where(measured,.85,.15); confidence[(labels==ROAD)&measured]=.95
    return FusionMap(fill,slope,traversability,confidence,blocked)

def occupancy_grid(fusion: FusionMap):
    """ROS OccupancyGrid-compatible values: -1 unknown, 0 free, 100 occupied."""
    values=np.full(fusion.blocked.shape,-1,dtype=np.int8)
    known=fusion.confidence>=.8
    values[known & ~fusion.blocked]=0
    values[known & fusion.blocked]=100
    return values
