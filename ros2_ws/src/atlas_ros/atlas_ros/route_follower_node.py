"""Conservative path follower for the simulated ATLAS rover."""
from math import atan2, cos, hypot, pi


def wrap(angle):
    return (angle + pi) % (2 * pi) - pi


def quaternion_yaw(q):
    return atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry, Path
    from rclpy.node import Node

    class RouteFollower(Node):
        def __init__(self):
            super().__init__('atlas_route_follower')
            self.declare_parameter('linear_speed_mps', .55)
            self.declare_parameter('lookahead_m', .75)
            self.path, self.pose, self.cursor = [], None, 0
            self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
            self.create_subscription(Path, '/atlas/route', self.on_path, 10)
            self.create_subscription(Odometry, '/odom', self.on_odom, 10)
            self.create_timer(.05, self.tick)

        def on_path(self, message):
            self.path = [(p.pose.position.x, p.pose.position.y) for p in message.poses]
            self.cursor = 0
            self.get_logger().info(f'Received route with {len(self.path)} waypoints.')

        def on_odom(self, message):
            pose = message.pose.pose
            self.pose = pose.position.x, pose.position.y, quaternion_yaw(pose.orientation)

        def tick(self):
            command = Twist()
            if not self.pose or not self.path:
                self.cmd.publish(command)
                return
            x, y, yaw = self.pose
            lookahead = float(self.get_parameter('lookahead_m').value)
            while self.cursor < len(self.path) - 1 and hypot(self.path[self.cursor][0] - x, self.path[self.cursor][1] - y) < lookahead:
                self.cursor += 1
            target_x, target_y = self.path[self.cursor]
            distance = hypot(target_x - x, target_y - y)
            if self.cursor == len(self.path) - 1 and distance < .2:
                self.path = []
                self.get_logger().info('Route complete.')
                self.cmd.publish(command)
                return
            error = wrap(atan2(target_y - y, target_x - x) - yaw)
            command.linear.x = float(self.get_parameter('linear_speed_mps').value) * max(.15, cos(error))
            command.angular.z = max(-1.1, min(1.1, 1.8 * error))
            self.cmd.publish(command)

    rclpy.init()
    node = RouteFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
