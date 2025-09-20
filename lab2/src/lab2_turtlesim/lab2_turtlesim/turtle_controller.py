import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtleController(Node):
    def __init__(self, turtle_name: str):
        super().__init__('turtle_controller')
        self.turtle_name = turtle_name
        topic = f'/{turtle_name}/cmd_vel'
        self.publisher = self.create_publisher(Twist, topic, 10)
        self.get_logger().info(f'Controlling {turtle_name} on topic {topic}')

    def send_command(self, linear_x: float = 0.0, angular_z: float = 0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run lab2_turtlesim turtle_controller <turtle_name>")
        sys.exit(1)

    turtle_name = sys.argv[1]
    node = TurtleController(turtle_name)
    try:
        while rclpy.ok():
            try:
                command = input("Enter command (w/a/s/d for movement, q to quit): ").strip().lower()
            except EOFError:
                break

            if command == 'w':
                node.send_command(linear_x=1.0, angular_z=0.0)
            elif command == 's':
                node.send_command(linear_x=-1.0, angular_z=0.0)
            elif command == 'a':
                node.send_command(linear_x=0.0, angular_z=1.0)
            elif command == 'd':
                node.send_command(linear_x=0.0, angular_z=-1.0)
            elif command == 'q':
                break
            else:
                print("Invalid command. Use w/a/s/d for movement, q to quit.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
