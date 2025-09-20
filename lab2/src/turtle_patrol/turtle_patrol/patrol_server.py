import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute
from turtle_patrol_interface.srv import Patrol


class PatrolServer(Node):
    def __init__(self):
        super().__init__('patrol_server')
        self.srv = self.create_service(Patrol, 'turtle_patrol', self.patrol_callback)

        self.turtle_publishers = {}
        self.get_logger().info('Patrol service ready.')



    def patrol_callback(self, request: Patrol.Request, response: Patrol.Response):
        turtle_name = request.turtle_name
        topic = f'/{turtle_name}/cmd_vel'

        if turtle_name not in self.turtle_publishers:
            self.turtle_publishers[turtle_name] = self.create_publisher(Twist, topic, 10)

        teleport_client = self.create_client(TeleportAbsolute, f'/{turtle_name}/teleport_absolute')
        if not teleport_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Teleport service for {turtle_name} not available.")
            return response
        teleport_req = TeleportAbsolute.Request()
        teleport_req.x = request.x
        teleport_req.y = request.y
        teleport_req.theta = request.theta
        teleport_client.call_async(teleport_req)

        twist = Twist()
        twist.linear.x = request.vel
        twist.angular.z = request.omega
        self.turtle_publishers[turtle_name].publish(twist)

        response.cmd = twist
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PatrolServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
