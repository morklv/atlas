"""Conservative PointCloud2-to-OccupancyGrid terrain node for ROS 2 Jazzy."""
import numpy as np

def grid_from_points(points, resolution_m=.5, size_m=48.):
    n=round(size_m/resolution_m); grid=np.full((n,n),-1,dtype=np.int8)
    points=np.asarray(points,dtype=float)
    valid=np.isfinite(points).all(axis=1)&(points[:,0]>=0)&(points[:,0]<size_m)&(points[:,1]>=0)&(points[:,1]<size_m)
    for x,y,z in points[valid]:
        ix,iy=int(x/resolution_m),int(y/resolution_m)
        grid[iy,ix]=100 if z>.35 else 0
    return grid

def main():
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    class NodeImpl(Node):
        def __init__(self):
            super().__init__('atlas_terrain_fusion'); self.declare_parameter('resolution_m',.5); self.declare_parameter('size_m',48.)
            self.pub=self.create_publisher(OccupancyGrid,'/atlas/terrain_occupancy',10); self.create_subscription(PointCloud2,'/points',self.on_cloud,10)
        def on_cloud(self,msg):
            r=float(self.get_parameter('resolution_m').value); size=float(self.get_parameter('size_m').value)
            points=list(point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)); grid=grid_from_points(points,r,size)
            out=OccupancyGrid(); out.header=msg.header; out.info.resolution=r; out.info.width=out.info.height=grid.shape[0]; out.info.origin.orientation.w=1.; out.data=grid.ravel().tolist(); self.pub.publish(out)
    rclpy.init(); node=NodeImpl(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
