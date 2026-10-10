#!/usr/bin/env python3
"""
Drive the simulated hexapod in each direction and check it moves the right way.

Run while hexapod_simulation.launch.py is up (ideally with world:=empty_world):
    ros2 run hexapod_bringup sim_direction_check.py --world empty_world

Directions follow ROS conventions for /cmd_vel: +linear.x forward (towards the cameras),
+linear.y left in strafe mode, positive yaw anticlockwise. In normal mode, linear.y turns.
Each case starts from the origin. Exits non-zero if any case moves the wrong way.
"""
import argparse
import math
import re
import subprocess
import sys

import rclpy
from geometry_msgs.msg import Twist
from hexapod_interfaces.srv import GetCapabilities, SetMode
from rclpy.node import Node
from rclpy.parameter import Parameter

MODE_NORMAL = 0
MODE_STRAFE = 1


class DirectionCheck(Node):
    """Command each direction in Gazebo and compare the body's motion with what was asked."""

    def __init__(self, world):
        super().__init__('sim_direction_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.world = world
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.mode_client = self.create_client(SetMode, 'set_mode')
        self.caps_client = self.create_client(GetCapabilities, 'get_capabilities')

    def now(self):
        """Simulation time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def call(self, client, request):
        """Call a service and wait for the reply."""
        client.wait_for_service()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def pose(self):
        """Robot position (x, y) and yaw in the world, from Gazebo."""
        out = subprocess.run(['gz', 'model', '-m', 'hexapod', '-p'], capture_output=True, text=True).stdout
        xyz, rpy = re.findall(r'\[([-\d.e ]+)\]', out.split('Pose [ XYZ')[1])[:2]
        x, y, _ = (float(v) for v in xyz.split())
        return x, y, float(rpy.split()[2])

    def reset(self):
        """Put the robot back at the origin, facing the same way each time."""
        subprocess.run(['gz', 'service', '-s', f'/world/{self.world}/set_pose',
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean', '--timeout', '2000',
                        '--req', 'name: "hexapod", position: {x: 0, y: 0, z: 0.13}, orientation: {w: 1}'],
                       capture_output=True)
        self.drive(Twist(), 2.0)

    def drive(self, twist, duration):
        """Publish a velocity command for a duration of sim time."""
        start = self.now()
        while self.now() - start < duration:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_case(self, name, mode, twist, expect):
        """
        Drive one case and check the motion in the robot's own frame.

        expect is 'forward', 'backward', 'left', 'right', 'turn_left' or 'turn_right'.
        Returns True if the robot moved mainly in the expected direction.
        """
        self.call(self.mode_client, SetMode.Request(mode=mode))
        self.reset()
        self.drive(twist, 1.5)  # let the gait settle
        x0, y0, yaw0 = self.pose()
        t0 = self.now()
        self.drive(twist, 3.5)
        x1, y1, yaw1 = self.pose()
        elapsed = self.now() - t0
        self.drive(Twist(), 0.5)

        # Body frame: +y forward, +x right
        dx, dy = x1 - x0, y1 - y0
        forward = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
        right = dx * math.cos(yaw0) + dy * math.sin(yaw0)
        turn = math.remainder(yaw1 - yaw0, 2 * math.pi)

        moves = {
            'forward': forward, 'backward': -forward,
            'right': right, 'left': -right,
            'turn_left': turn, 'turn_right': -turn,
        }
        linear = math.hypot(forward, right)
        if expect.startswith('turn'):
            ok = moves[expect] > 0.05 and linear < 0.1
            detail = f'turned {math.degrees(turn):+.1f} deg ({turn / elapsed:+.3f} rad/s), drifted {linear:.3f} m'
        else:
            ok = moves[expect] > 0.8 * linear and moves[expect] > 0.05
            detail = (f'forward {forward:+.3f} m, right {right:+.3f} m, '
                      f'turned {math.degrees(turn):+.1f} deg, speed {linear / elapsed:.3f} m/s')
        print(f'{"PASS" if ok else "FAIL"}  {name:<22} {detail}')
        return ok


def main():
    """Run every direction case and report the result."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--world', default='empty_world', help='Gazebo world name (default: empty_world)')
    args = parser.parse_args()

    rclpy.init()
    node = DirectionCheck(args.world)
    while node.now() == 0.0:  # wait for the first /clock message
        rclpy.spin_once(node, timeout_sec=0.1)

    caps = node.call(node.caps_client, GetCapabilities.Request())
    v = 0.5 * caps.max_linear_vel
    w = 0.5 * caps.max_angular_vel
    print(f'Testing at {v:.3f} m/s and {w:.3f} rad/s (half the top speeds)')

    def twist(x=0.0, y=0.0, z=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = x, y, z
        return t

    cases = [
        ('forward', MODE_NORMAL, twist(x=v), 'forward'),
        ('backward', MODE_NORMAL, twist(x=-v), 'backward'),
        ('turn left (normal)', MODE_NORMAL, twist(y=v), 'turn_left'),
        ('turn right (normal)', MODE_NORMAL, twist(y=-v), 'turn_right'),
        ('strafe left', MODE_STRAFE, twist(y=v), 'left'),
        ('strafe right', MODE_STRAFE, twist(y=-v), 'right'),
        ('turn left (strafe)', MODE_STRAFE, twist(z=w), 'turn_left'),
        ('turn right (strafe)', MODE_STRAFE, twist(z=-w), 'turn_right'),
    ]
    results = [node.run_case(*case) for case in cases]
    node.call(node.mode_client, SetMode.Request(mode=MODE_NORMAL))

    passed = sum(results)
    print(f'{passed}/{len(results)} directions correct')
    rclpy.shutdown()
    sys.exit(0 if passed == len(results) else 1)


if __name__ == '__main__':
    main()
