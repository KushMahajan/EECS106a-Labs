# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from my_chatter_msgs.msg import TimestampString


class UserInputSubscriber(Node):

    def __init__(self):
        super().__init__('user_input_subscriber')
        self.subscription = self.create_subscription(
            TimestampString,
            '/user_messages',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        recv_time = self.get_clock().now().nanoseconds
        self.get_logger().info('Message: "%s", Sent at: "%s", Received at: "%s"' % (msg.data, msg.timestamp, recv_time))


def main(args=None):
    rclpy.init(args=args)

    user_input_subscriber = UserInputSubscriber()

    rclpy.spin(user_input_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    user_input_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
