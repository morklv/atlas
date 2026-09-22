import unittest
import numpy as np
from sentinel.lidar_fusion import ROAD, FOREST, WATER, fuse, occupancy_grid

class LidarFusionTests(unittest.TestCase):
    def test_semantics_and_measured_slope_drive_traversability(self):
        labels=np.zeros((5,5),dtype=np.uint8); labels[2,:]=ROAD; labels[1,2]=FOREST; labels[3,2]=WATER
        height=np.zeros((5,5)); observed=np.ones((5,5),dtype=bool)
        result=fuse(labels,height,observed,1)
        self.assertGreater(result.traversability[2,1],.9)
        self.assertTrue(result.blocked[1,2]); self.assertTrue(result.blocked[3,2])
        self.assertEqual(occupancy_grid(result)[2,1],0)
        self.assertEqual(occupancy_grid(result)[1,2],100)
    def test_unobserved_space_stays_unknown_and_blocked(self):
        labels=np.full((3,3),ROAD,dtype=np.uint8); observed=np.ones((3,3),dtype=bool); observed[1,1]=False
        result=fuse(labels,np.zeros((3,3)),observed,.5)
        self.assertTrue(result.blocked[1,1]); self.assertEqual(occupancy_grid(result)[1,1],-1)
