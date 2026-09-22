"""Publish a repeatable test route for the ATLAS Gazebo scenario."""


def main():
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from rclpy.node import Node

    class DemoRoute(Node):
        def __init__(self):
            super().__init__('atlas_demo_route')
            self.pub = self.create_publisher(Path, '/atlas/route', 1)
            self.sent = False
            self.create_timer(2., self.publish_once)

        def publish_once(self):
            if self.sent:
                return
            route = Path()
            route.header.frame_id = 'world'
            # A route that follows the dark road corridor through the scenario.
            for x, y in [(1., 1.), (3., 1.), (5., 1.), (7., 1.), (8., 2.5), (8., 5.), (8., 7.5)]:
                waypoint = PoseStamped()
                waypoint.header.frame_id = 'world'
                waypoint.pose.position.x = x
                waypoint.pose.position.y = y
                waypoint.pose.orientation.w = 1.
                route.poses.append(waypoint)
            self.pub.publish(route)
            self.sent = True
            self.get_logger().info('Published ATLAS demonstration route.')

    rclpy.init()
    node = DemoRoute()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
