#
#   Copyright 2021 R. Kent James <kent@caspia.com>
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.

import math

from fqdemo_msgs.msg import NumPwrData, NumPwrResult
from rclpy.node import Node


class PySubPub(Node):
    """
    Apply a sample filter to an incoming topic, publishing result.

    Listens for a message with a number and power. Publishes a message with that number to that
    power, and to that root.
    """

    def __init__(self):
        super().__init__('pysubpub')
        self.publisher = self.create_publisher(NumPwrResult, 'power_result', 10)
        self.subscriber = self.create_subscription(
          NumPwrData, 'num_power', self.topic_callback, 10
        )
        timer_period_seconds = 0.5
        self.timer = self.create_timer(timer_period_seconds, self.timer_callback)

    @staticmethod
    def apply_powers(number, power):
        to_power = math.pow(number, power)
        to_root = math.pow(number, 1. / power)
        return ((to_power, to_root))

    def timer_callback(self):
        msg = NumPwrResult()
        msg.to_power = 0.0
        msg.to_root = 0.0
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing to_power: {msg.to_power}, to_root: {msg.to_root}')

    def topic_callback(self, msg):
        self.get_logger().info(f'I heard {msg}')
        response_msg = NumPwrResult()
        (response_msg.to_power, response_msg.to_root) = self.apply_powers(msg.num, msg.power)
        self.publisher.publish(response_msg)
        self.get_logger().info(f'Publishing to_power: {response_msg.to_power}, to_root: {response_msg.to_root}')
