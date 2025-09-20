import sys
import rclpy
from rclpy.node import Node
from turtle_patrol_interface.srv import Patrol

class PatrolClient(Node):
    def __init__(self):
        super().__init__('patrol_client')
        self.client = self.create_client(Patrol, 'turtle_patrol')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for patrol service...')

    def send_request(self, turtle_name: str, vel: float, omega: float, x: float, y: float, theta: float):
        req = Patrol.Request()
        req.turtle_name = turtle_name
        req.x = x
        req.y = y
        req.theta = theta
        req.vel = vel
        req.omega = omega
        return self.client.call_async(req)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 7:
        print("Usage: ros2 run turtle_patrol patrol_client <turtle_name> <vel> <omega> <x> <y> <theta>")
        sys.exit(1)
        return
    
    turtle_name, vel, omega, x, y, theta = sys.argv[1:7]
    node = PatrolClient()
    future = node.send_request(turtle_name, float(vel), float(omega), float(x), float(y), float(theta))

    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        response = future.result()
        node.get_logger().info(f'Recieved Twist: linear={response.cmd.linear.x}, angular={response.cmd.angular.z}')
    else:
        node.get_logger().error('Service call failed')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()