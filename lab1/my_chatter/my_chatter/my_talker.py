import rclpy
from rclpy.node import Node
from my_chatter_msgs.msg import TimestampString

def main(args=None):
    rclpy.init(args=args)
    node = Node('my_talker')
    pub = node.create_publisher(TimestampString, '/user_messages', 10)
    try:
        while rclpy.ok():
            text = input("Please enter a line of text and press <Enter>: ")
            msg = TimestampString()
            msg.data = text
            msg.timestamp = node.get_clock().now().nanoseconds
            pub.publish(msg)
            node.get_logger().info(f'Published: "{msg.data}" at {msg.timestamp}')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
