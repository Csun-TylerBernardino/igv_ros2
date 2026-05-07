#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP_TEXT = """
Control your differential drive robot
-------------------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease both speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

MOVE_BINDINGS = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}

SPEED_BINDINGS = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}


def get_key(settings) -> str:
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__('differential_drive_teleop')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('scale_linear', 0.2)
        self.declare_parameter('scale_angular', 1.0)
        self.declare_parameter('linear_step', 0.02)
        self.declare_parameter('angular_step', 0.1)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.speed = float(self.get_parameter('scale_linear').value)
        self.turn = float(self.get_parameter('scale_angular').value)
        self.linear_step = float(self.get_parameter('linear_step').value)
        self.angular_step = float(self.get_parameter('angular_step').value)

        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.x = 0
        self.th = 0
        self.status = 0
        self.count = 0
        self.target_speed = 0.0
        self.target_turn = 0.0
        self.control_speed = 0.0
        self.control_turn = 0.0

    def publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)

    def stop(self) -> None:
        self.publish_twist(0.0, 0.0)

    def format_vels(self) -> str:
        return f'currently:\tspeed {self.speed}\tturn {self.turn}'

    def run(self) -> None:
        settings = termios.tcgetattr(sys.stdin)
        print(HELP_TEXT)
        print(self.format_vels())

        try:
            while rclpy.ok():
                key = get_key(settings)
                if key in MOVE_BINDINGS:
                    self.x, self.th = MOVE_BINDINGS[key]
                    self.count = 0
                elif key in SPEED_BINDINGS:
                    self.speed *= SPEED_BINDINGS[key][0]
                    self.turn *= SPEED_BINDINGS[key][1]
                    self.count = 0
                    print(self.format_vels())
                    if self.status == 14:
                        print(HELP_TEXT)
                    self.status = (self.status + 1) % 15
                elif key in (' ', 'k'):
                    self.x = 0
                    self.th = 0
                    self.control_speed = 0.0
                    self.control_turn = 0.0
                else:
                    self.count += 1
                    if self.count > 4:
                        self.x = 0
                        self.th = 0
                    if key == '\x03':
                        break

                self.target_speed = self.speed * self.x
                self.target_turn = self.turn * self.th

                if self.target_speed > self.control_speed:
                    self.control_speed = min(self.target_speed, self.control_speed + self.linear_step)
                elif self.target_speed < self.control_speed:
                    self.control_speed = max(self.target_speed, self.control_speed - self.linear_step)
                else:
                    self.control_speed = self.target_speed

                if self.target_turn > self.control_turn:
                    self.control_turn = min(self.target_turn, self.control_turn + self.angular_step)
                elif self.target_turn < self.control_turn:
                    self.control_turn = max(self.target_turn, self.control_turn - self.angular_step)
                else:
                    self.control_turn = self.target_turn

                self.publish_twist(self.control_speed, self.control_turn)
        except Exception as exc:  # pragma: no cover - best effort terminal cleanup
            self.get_logger().error(f'Teleop stopped due to: {exc}')
        finally:
            self.stop()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main() -> None:
    rclpy.init()
    node = TeleopNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
