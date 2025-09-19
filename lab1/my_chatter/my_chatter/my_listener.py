import rclpy
from rclpy.node import Node
from my_chatter_msgs.msg import TimestampString

def main(args=None):
    rclpy.init(args=args)
    node = Node('my_listener')

    def callback(msg):
        recv_time = node.get_clock().now().nanoseconds
        print(f'Message: {msg.data}, Sent at: {msg.timestamp}, Received at: {recv_time}')

    node.create_subscription(TimestampString, '/user_messages', callback, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
